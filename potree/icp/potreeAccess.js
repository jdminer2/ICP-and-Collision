import { OctreeGeometryNode } from "../src/modules/loader/2.0/OctreeGeometry";
import { BufferGeometry, BufferAttribute, Euler, Vector3 } from "../libs/three.js/build/three.module";
  // Returns all points in the pointcloud from the nodes at the specified depth, if they are contained within the given clipVolume.
  export async function getPointsFromPointcloudUsingClipVolume(clipVolume,depth,pointcloud) {
    // Obtain the 6 normals, 3 from the clipVolume's faces and 3 from the pointcloud's nodes' faces.
    let normals = [
      new Vector3(1,0,0).applyEuler(clipVolume.rotation),
      new Vector3(0,1,0).applyEuler(clipVolume.rotation),
      new Vector3(0,0,1).applyEuler(clipVolume.rotation),
      new Vector3(1,0,0).applyEuler(pointcloud.rotation),
      new Vector3(0,1,0).applyEuler(pointcloud.rotation),
      new Vector3(0,0,1).applyEuler(pointcloud.rotation)
    ];

    let clipVolumeProjections = getBoxProjections(normals, clipVolume, clipVolume);

    return await getPointsRecursive(await getRootNode(pointcloud), depth, pointcloud, 
      volumeIntersectsNode, {normals, clipVolumeProjections}, 
      volumeContainsPoint, {normals, clipVolumeProjections})
  }

  // Selection method for nodes which only takes points inside a clipping volume.
  // According to the separating axis theorem, for two convex polyhedra, if the projections of the polyhedra overlap
  // for each normal of both polyhedra's faces, then and only then do the polyhedra intersect. https://dyn4j.org/2010/01/sat/
  function volumeIntersectsNode({normals, clipVolumeProjections}, node, pointcloud) {
    let nodeProjections = getBoxProjections(normals, node, pointcloud);

    return normals.every((normal,i) =>
      // If the A's min <= B's max and A's max >= B's min, then A and B are overlapping.
      clipVolumeProjections[i][0] <= nodeProjections[i][1] && clipVolumeProjections[i][1] >= nodeProjections[i][0]
    )
  }

  // Selection method for points which only takes points inside a clipping volume.
  function volumeContainsPoint({normals, clipVolumeProjections}, point) {
    return normals.every((normal,i) => {
      let projectedPoint = point.dot(normal);
      return clipVolumeProjections[i][0] <= projectedPoint && projectedPoint <= clipVolumeProjections[i][1];
    });
  }

  // If the normal was extended into an infinite line, and the box was projected onto this line, 
  // this returns the interval that the projection covers. (Repeated for each normal).
  function getBoxProjections(normals, boundingBoxHolder, transformationHolder) {    
    let cornerPoints = [0,1,2,3,4,5,6,7].map(i => {
      let x = i % 2;
      let y = Math.floor(i / 2) % 2;
      let z = Math.floor(i / 4);
      return new Vector3(
        x ? boundingBoxHolder.boundingBox.max.x : boundingBoxHolder.boundingBox.min.x,
        y ? boundingBoxHolder.boundingBox.max.y : boundingBoxHolder.boundingBox.min.y,
        z ? boundingBoxHolder.boundingBox.max.z : boundingBoxHolder.boundingBox.min.z
      ).applyEuler(transformationHolder.rotation).multiply(transformationHolder.scale).add(transformationHolder.position);
    })

    // Gets the [lowest,highest] from the dot products of the cornerPoints with the normal. Repeat for each normal.
    return normals.map(normal =>
      cornerPoints.reduce((currentProjection, cornerPoint) => {
        let dotProduct = cornerPoint.dot(normal);
        return [Math.min(dotProduct, currentProjection[0]), Math.max(dotProduct, currentProjection[1])]
      }, [Infinity, -Infinity])
    );
  }

  export async function getPointsFromPointcloudUsingMaxDistance(maxDistance,targetPoints,depth,pointcloud) {
    let targetPointObjects = targetPoints.map(point => new Vector3(...point))
    return await getPointsRecursive(await getRootNode(pointcloud), depth, pointcloud, 
      nodeInMaxDistance, {maxDistance, targetPointObjects}, 
      pointInMaxDistance, {maxDistance, targetPointObjects}
    )
  }

  function nodeInMaxDistance({maxDistance, targetPointObjects}, node, pointcloud) {
    // I want to compare the bounding box and the target point in a frame of reference where the bounding box is axis-aligned,
    // And I want to measure maxDistance in target point's units.
    // So I apply scale to bounding box, but not rotation. This means I must apply reverse rotation and position to target point.
    // Scale can be applied before rotation. All that matters is that position is applied last, or reverse position is applied first.
    let min = node.boundingBox.min.clone().multiply(pointcloud.scale);
    let max = node.boundingBox.max.clone().multiply(pointcloud.scale);

    // This only matters if a component of pointcloud.scale is negative.
    let newMin = new Vector3(Math.min(min.x,max.x), Math.min(min.y,max.y), Math.min(min.z,max.z))
    let newMax = new Vector3(Math.max(min.x,max.x), Math.max(min.y,max.y), Math.max(min.z,max.z));

    let reverseOrder = pointcloud.rotation.order[2] + pointcloud.rotation.order[1] + pointcloud.rotation.order[0];
    let reverseRotation = new Euler(-pointcloud.rotation.x, -pointcloud.rotation.y, -pointcloud.rotation.z, reverseOrder)

    return targetPointObjects.some(targetPoint => {
      let newTargetPoint = targetPoint.clone().sub(pointcloud.position).applyEuler(reverseRotation);
      let x = Math.max(0, newTargetPoint.x - newMax.x, newMin.x - newTargetPoint.x);
      let y = Math.max(0, newTargetPoint.y - newMax.y, newMin.y - newTargetPoint.y);
      let z = Math.max(0, newTargetPoint.z - newMax.z, newMin.z - newTargetPoint.z);
      return Math.sqrt(x**2 + y**2 + z**2) <= maxDistance;
    })
  }

  function pointInMaxDistance({maxDistance, targetPointObjects}, point) {
    return targetPointObjects.some(targetPoint => targetPoint.distanceTo(point) <= maxDistance);
  }

  // Returns the points in the specified node, and its descendants to the specified depth. 
  // Filters nodes according to nodeSelectionMethod, to avoid searching nodes that will have no valid points, reducing the amount of searching.
  // Filters points according to pointSelectionMethod, to keep only valid points.
  // PrecomputedValues are values used in the SelectionMethods which would be too expensive to compute every time the SelectionMethod is run.
  // They are computed earlier, and passed into the functions every time.
  async function getPointsRecursive(node, depth, pointcloud, 
    nodeSelectionMethod, nodeSelectionPrecomputedValues, 
    pointSelectionMethod, pointSelectionPrecomputedValues
  ) {
    // Filter nodes using the nodeSelectionMethod, and do not search deeper in them unless they are selected.
    if(!nodeSelectionMethod(nodeSelectionPrecomputedValues, node, pointcloud))
      return [];
    loadNode(node);
    // If reached the target depth, take the points.
    if(depth == 0) {
      // Extract points from the node
      let position = await waitForDef(()=>node.geometry?.attributes?.position);
      let points = Array(position.array.length/position.itemSize);
      for(let i = 0; i < position.array.length/position.itemSize; i++) {
        // Transform the point data according to bounding box and pointcloud's transformations.
        points[i] = new Vector3(...(position.array.slice(i*position.itemSize, i*position.itemSize + 3))).add(node.boundingBox.min)
          .applyEuler(pointcloud.rotation).multiply(pointcloud.scale).add(pointcloud.position);
      }
      // Filter points using the pointSelectionMethod.
      return points.filter(point => pointSelectionMethod(pointSelectionPrecomputedValues, point)).map(point => point.toArray());
    }
    // If not reached the target depth, recursively take the node's children.
    if(depth > 0) {
      let childArray = await waitForDef(()=>node.children).then(children=>
        Object.values(children).map(child=>getPointsRecursive(child, depth-1, pointcloud,
          nodeSelectionMethod, nodeSelectionPrecomputedValues, 
          pointSelectionMethod, pointSelectionPrecomputedValues
        ))
      );
      return (await Promise.all(childArray)).flat();
    }
  }

  // Copies the root of the pointcloud so that loading its children doesn't affect which points are loaded in the rendered scene
  // It seems like many colorless potree points are added to the scene by this algorithm, so maybe there is a problem here or in loadNode.
  async function getRootNode(pointcloud) {
    return await waitForDef(()=> {
      let val = pointcloud?.pcoGeometry?.root?.loaded;
      if(val !== false && val !== undefined)
        return pointcloud.pcoGeometry.root;
      else
        return undefined;
    }).then(root=>{
      let nodeGeom = {...root.octreeGeometry}
      nodeGeom.loader={...nodeGeom.loader}

      let node = new OctreeGeometryNode("r", nodeGeom, root.boundingBox)
      node.level = root.level;
      node.nodeType = root.nodeType;
      node.hierarchyByteOffset = root.hierarchyByteOffset;
      node.hierarchyByteSize = root.hierarchyByteSize;
      node.children = root.children;
      node.hasChildren = root.hasChildren;
      node.spacing = root.spacing;
      node.byteOffset = root.byteOffset;
      node.byteSize = root.byteSize;
      node.numPoints = root.numPoints;
      return node
    })
  }

  // Asynchronously waits for the value to become defined. Necessary because pointcloud loading is strange and some properties don't exist initially.
  async function waitForDef(getter) {
    let val = getter();
    while(val === undefined) {
      await new Promise(e=>setTimeout(e,0));
      val = getter();
    }
    return val
  }

  // Copy of NodeLoader.load, without being blocked by the loaded nodes limit, and with some details ignored.
  async function loadNode(node) {
    try{
      if(node.nodeType === 2){
        await node.octreeGeometry.loader.loadHierarchy(node);
      }

      let {byteOffset, byteSize} = node;


      let urlOctree = `${node.octreeGeometry.loader.url}/../octree.bin`;

      let first = byteOffset;
      let last = byteOffset + byteSize - 1n;

      let buffer;

      if(byteSize === 0n){
        buffer = new ArrayBuffer(0);
        console.warn(`loaded node with 0 bytes: ${node.name}`);
      }else {
        let response = await fetch(urlOctree, {
          headers: {
            'content-type': 'multipart/byteranges',
            'Range': `bytes=${first}-${last}`,
          },
        });

        buffer = await response.arrayBuffer();
      }

      let workerPath;
      if(node.octreeGeometry.loader.metadata.encoding === "BROTLI"){
        workerPath = Potree.scriptPath + '/workers/2.0/DecoderWorker_brotli.js';
      }else {
        workerPath = Potree.scriptPath + '/workers/2.0/DecoderWorker.js';
      }

      let worker = Potree.workerPool.getWorker(workerPath);

      worker.onmessage = function (e) {

        let data = e.data;
        let buffers = data.attributeBuffers;

        Potree.workerPool.returnWorker(workerPath, worker);

        let geometry = new BufferGeometry();
        
        for(let property in buffers)
          if(property === "position"){
            let buffer = buffers[property].buffer;
            geometry.setAttribute('position', new BufferAttribute(new Float32Array(buffer), 3));
          }

        node.density = data.density;
        node.geometry = geometry;
      };

      let pointAttributes = node.octreeGeometry.pointAttributes;
      let scale = node.octreeGeometry.scale;

      let box = node.boundingBox;
      let min = node.octreeGeometry.offset.clone().add(box.min);
      let size = box.max.clone().sub(box.min);
      let max = min.clone().add(size);
      let numPoints = node.numPoints;

      let offset = node.octreeGeometry.loader.offset;

      let message = {
        name: node.name,
        buffer: buffer,
        pointAttributes: pointAttributes,
        scale: scale,
        min: min,
        max: max,
        size: size,
        offset: offset,
        numPoints: numPoints
      };

      worker.postMessage(message, [message.buffer]);
    }catch(e){
      console.log(`failed to load ${node.name}`);
      console.log(e);
      console.log(`trying again!`);
    }
  }