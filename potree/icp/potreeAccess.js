import { OctreeGeometryNode } from "../src/modules/loader/2.0/OctreeGeometry";
import { BufferGeometry, BufferAttribute, Euler, Vector3, Matrix4 } from "../libs/three.js/build/three.module";
import { Matrix as Mlmatrix } from 'ml-matrix';
import { waitForDef, matrix4ToMlmatrix } from "./utils"

  // Precomputed values for the clipVolume point filtering strategy.
  export function clipVolumePrecomputedValues(clipVolume,pointcloud) {
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

    return {normals, clipVolumeProjections};
  }
  
  // maxDistancePrecomputedValues should be {maxDistance, targetPoints} where targetPoints is in Vector3 form.

  // Filtering method for nodes which only accepts nodes that may have points inside a box volume.
  // According to the separating axis theorem, for two convex polyhedra, if the projections of the polyhedra overlap
  // for each normal of both polyhedra's faces, then and only then do the polyhedra intersect. https://dyn4j.org/2010/01/sat/
  export function clipVolumeNodeFilterMethod({normals, clipVolumeProjections}, node, pointcloud) {
    let nodeProjections = getBoxProjections(normals, node, pointcloud);

    return normals.every((normal,i) =>
      // If the A's min <= B's max and A's max >= B's min, then A and B are overlapping.
      clipVolumeProjections[i][0] <= nodeProjections[i][1] && clipVolumeProjections[i][1] >= nodeProjections[i][0]
    )
  }

  // Filtering method for nodes which only accepts nodes that may have points within maxDistance of a target point.
  export function maxDistanceNodeFilterMethod({maxDistance, targetPoints}, node, pointcloud) {
    // I want to compare the bounding box and the target point in a frame of reference where the bounding box is axis-aligned,
    // And I want to measure maxDistance in target point's units.
    // So I apply scale to bounding box, but not rotation. This means I must apply reverse rotation and position to target point.
    // Scale can be applied before rotation. All that matters is that position is applied last, or reverse position is applied first.
    let min = node.boundingBox.min.clone().multiply(pointcloud.scale);
    let max = node.boundingBox.max.clone().multiply(pointcloud.scale);

    // This only matters if a component of pointcloud.scale is negative.
    let newMin = new Vector3(Math.min(min.x,max.x), Math.min(min.y,max.y), Math.min(min.z,max.z))
    let newMax = new Vector3(Math.max(min.x,max.x), Math.max(min.y,max.y), Math.max(min.z,max.z));

    let reverseOrder = pointcloud.rotation.order[2] + pointcloud.rotation.order[1] + pointcloud.rotation.order[0]; // String reversal.
    let reverseRotation = new Euler(-pointcloud.rotation.x, -pointcloud.rotation.y, -pointcloud.rotation.z, reverseOrder)

    return targetPoints.some(targetPoint => {
      let newTargetPoint = targetPoint.clone().sub(pointcloud.position).applyEuler(reverseRotation);
      let x = Math.max(0, newTargetPoint.x - newMax.x, newMin.x - newTargetPoint.x);
      let y = Math.max(0, newTargetPoint.y - newMax.y, newMin.y - newTargetPoint.y);
      let z = Math.max(0, newTargetPoint.z - newMax.z, newMin.z - newTargetPoint.z);
      return Math.sqrt(x**2 + y**2 + z**2) <= maxDistance;
    })
  }

  // Filtering method for points which only accepts points inside a box volume. Expects point to be a Vector3.
  export function clipVolumePointFilterMethod({normals, clipVolumeProjections}, point) {
    return normals.every((normal,i) => {
      let projectedPoint = point.dot(normal);
      return clipVolumeProjections[i][0] <= projectedPoint && projectedPoint <= clipVolumeProjections[i][1];
    });
  }

  // Filtering method for points which only accepts points within maxDistance of a target point. Expects point and targetPoints to be Vector3s.
  export function maxDistancePointFilterMethod({maxDistance, targetPoints}, point) {
    return targetPoints.some(targetPoint =>
      (targetPoint.x - point[0])**2 + (targetPoint.y - point[1])**2 + (targetPoint.z - point[2])**2 < maxDistance**2
    );
  }

  // If the normal was extended into an infinite line, and the box was projected onto this line, 
  // this returns the interval that the projection covers. (Repeated for each normal).
  // Helper function for clipVolume methods.
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

  // Returns an array of the child nodes of the nodes in prevDepth, filtered according to nodeFilterMethod, and loaded so they can be accessed.
  // Precomputed values are values that are the same for all nodes/points and are expensive to compute every time. 
  // It is better to compute them just once and pass them into the function.
  export async function getNextDepthOfNodes(prevDepth, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues) {
    // Async map to get children of prevDepth.
    let newDepth = (await Promise.all(prevDepth.map(async node =>
      Object.values(await waitForDef(() => node.children))
    )))
    // Flatten into a 1D array.
    .flat()
    // Keep only the nodes that pass nodeFilterMethod.
    .filter(node => nodeFilterMethod(nodeFilterPrecomputedValues, node, pointcloud));
    // Add to newDepth.
    return newDepth;
  }

  export async function getPointcloudPoints(N, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues, pointFilterMethod, pointFilterPrecomputedValues) {
    let nodes = [];
    let depth = 0;
    let points = [];
    while(points.length < N) {
      if(depth == 0)
        nodes = [await getRootNode(sourcePointcloud)];
      else
        nodes = await getNextDepthOfNodes(nodes, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues);
      
      if(nodes.length == 0)
        break;
      
      points = points.concat(await getPointsFromNodes(nodes,pointcloud,pointFilterMethod,pointFilterPrecomputedValues));
    }
    return points;
  }

  // Returns an array of Vector3s representing points, taken from the given nodes, filtered according to pointFilterMethod.
  export async function getPointsFromNodes(nodes, pointcloud, pointFilterMethod, pointFilterPrecomputedValues) {
    let pointcloudMatrix =  
    new Matrix4().makeTranslation(pointcloud.position.x,pointcloud.position.y,pointcloud.position.z).multiply(
      new Matrix4().makeScale(pointcloud.scale.x,pointcloud.scale.y,pointcloud.scale.z).multiply(
        new Matrix4().makeRotationFromEuler(pointcloud.rotation)
    ))
    return new Mlmatrix(
      // Maps nodes to an array of promises for their points, then await to get an array of every array of points.
      (await Promise.all(nodes.map(async node => {
        loadNode(node);
        // The transformation matrix to apply to the points of this node. The transformation and points matrices are transposed because it's faster that way.
        let nodeTransformation = matrix4ToMlmatrix(
          new Matrix4().multiplyMatrices(
            pointcloudMatrix, 
            new Matrix4().makeTranslation(node.boundingBox.min.x,node.boundingBox.min.y,node.boundingBox.min.z)
        )).transpose();

        /* TEST CODE: Both of these should result in the same values. Currently passing. * /
          console.log(new Vector3(1,4,3.7).applyMatrix4(new Matrix4().multiplyMatrices(
            pointcloudMatrix, 
            new Matrix4().makeTranslation(node.boundingBox.min.x,node.boundingBox.min.y,node.boundingBox.min.z)
          )));
          
          console.log(new Vector3(1,4,3.7).add(node.boundingBox.min).applyEuler(pointcloud.rotation).multiply(pointcloud.scale).add(pointcloud.position));
        /**/

        // Extract point data from the node.
        let position = await waitForDef(()=>node.geometry?.attributes?.position);
        let pointsMatrix = Mlmatrix.from1DArray(node.numPoints,3,position.array)
          // Add a column of ones for the 4D transformation matrix.
          .addColumn((new Array(node.numPoints)).fill(1));
        // Apply transformation matrix.
        let transformedPoints = pointsMatrix.mmul(nodeTransformation);
        return transformedPoints.data;
      })))
      // Flatten to get a single array of points.
      .flat()
      // Filter points according to pointFilterMethod.
      .filter(point => pointFilterMethod(pointFilterPrecomputedValues, point))
    )
  }

  // Copies the root of the pointcloud so that loading its children doesn't affect which points are loaded in the rendered scene
  // It seems like many colorless potree points are added to the scene by this algorithm, so maybe there is a problem here or in loadNode.
  export async function getRootNode(pointcloud) {
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