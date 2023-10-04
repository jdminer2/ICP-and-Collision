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

    // Project the clipVolume box onto each normal
    let clipVolumeProjections = getBoxProjections(normals, clipVolume, clipVolume);

    return {normals, clipVolumeProjections};
  }

  // Filtering method for nodes which only accepts nodes that may have points inside a box volume.
  export function clipVolumeNodeFilterMethod({normals, clipVolumeProjections}, node, pointcloud) {
    // Project the node bounding box onto each normal.
    let nodeProjections = getBoxProjections(normals, node, pointcloud);

    // Compare the projections and check for overlap. 
    // According to the separating axis theorem, two convex polyhedra intersect if and only if the projections of the polyhedra overlap
    // for every normal of both polyhedra's faces. https://dyn4j.org/2010/01/sat/
    return normals.every((_,i) =>
      // If the A's min <= B's max and A's max >= B's min, then A and B are overlapping.
      clipVolumeProjections[i][0] <= nodeProjections[i][1] && clipVolumeProjections[i][1] >= nodeProjections[i][0]
    )
  }

  // Filtering method for points which only accepts points inside a box volume. Expects point to be a Vector3.
  export function clipVolumePointFilterMethod({normals, clipVolumeProjections}, point) {
    return normals.every((normal,i) => {
      let projectedPoint = point.dot(normal);
      return clipVolumeProjections[i][0] <= projectedPoint && projectedPoint <= clipVolumeProjections[i][1];
    });
  }

  
  // maxDistancePrecomputedValues should be {squaredMaxDistance, targetKdTree} where targetPoints are in Vector3 form.
  
  // Filtering method for nodes which only accepts nodes that may have points within maxDistance of a target point.
  export function maxDistanceNodeFilterMethod({squaredMaxDistance, targetKdTree}, node, pointcloud) {
    if(squaredMaxDistance < 0)
      return false;
    
    let min = node.boundingBox.min.clone();
    let max = node.boundingBox.max.clone();
    
    // Once the original bounding box is transformed (including rotation) widenedBox will be a new axis-aligned box that surrounds the old box
    // such that any qualifying points must be inside it, and its size is as small as possible.
    let widenedBoxMin = {x:Infinity, y:Infinity, z:Infinity};
    let widenedBoxMax = {x:-Infinity, y:-Infinity, z:-Infinity};

    [0,1,2,3,4,5,6,7].forEach(i => {
      // Construct a different corner of the box for each number.
      let x = i % 2;
      let y = Math.floor(i / 2) % 2;
      let z = Math.floor(i / 4);
      let point = new Vector3(
        x ? max.x : min.x,
        y ? max.y : min.y,
        z ? max.z : min.z
      )
      // Transform the corner according to pointcloud's transformation.
      .multiply(pointcloud.scale).applyEuler(pointcloud.rotation).add(pointcloud.position);
    
      // Extend the widenedBox to contain it.
      ["x","y","z"].forEach(dim => {
        widenedBoxMin[dim] = Math.min(widenedBoxMin[dim],point[dim] - Math.sqrt(squaredMaxDistance));
        widenedBoxMax[dim] = Math.max(widenedBoxMax[dim],point[dim] + Math.sqrt(squaredMaxDistance));
      });
    })

    // Greatly reduce the size of the targetPoints to search by selecting only those in widenedBox.
    let candidatePoints = targetKdTree.subsection(widenedBoxMin, widenedBoxMax);

    // Apply scale to the bounding box, and reverse position and rotation to the targetPoints, because it's easier and equivalent to just applying scale then rotation then position to the bounding box.
    min.multiply(pointcloud.scale);
    max.multiply(pointcloud.scale);

    let reverseOrder = pointcloud.rotation.order[2] + pointcloud.rotation.order[1] + pointcloud.rotation.order[0]; // String reversal.
    let reverseRotation = new Euler(-pointcloud.rotation.x, -pointcloud.rotation.y, -pointcloud.rotation.z, reverseOrder);

    return candidatePoints.some(targetPoint => {
      let newTargetPoint = targetPoint.clone().sub(pointcloud.position).applyEuler(reverseRotation);
      let x = Math.max(0, newTargetPoint.x - max.x, min.x - newTargetPoint.x);
      let y = Math.max(0, newTargetPoint.y - max.y, min.y - newTargetPoint.y);
      let z = Math.max(0, newTargetPoint.z - max.z, min.z - newTargetPoint.z);
      return x**2 + y**2 + z**2 <= squaredMaxDistance;
    });
  }

  // Filtering method for points which only accepts points within maxDistance of a target point. Expects point to be Vector3 and targetKdTree to be kdTree of Vector3 points.
  export function maxDistancePointFilterMethod({squaredMaxDistance, targetKdTree}, point) {
    return targetKdTree.searchExistence(point,squaredMaxDistance);
  }

  // If the normal was extended into an infinite line, and the box was projected onto this line, 
  // this returns the interval on the line that the projection covers. (Repeated for each normal).
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

  // If depth == 0, use the pointcloud's root. Else, take the children of the previous array of nodes.
  // Filter according to nodeFilterMethod, and load so they can be accessed.
  // Precomputed values are values that are the same for all nodes/points and are expensive to compute every time. 
  // It is better to compute them just once and pass them into the function.
  export async function getNextDepthOfNodes(depth, prevNodes, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues) {
    let newNodes = [];
    if(depth == 0)
      newNodes = [await getRootNode(pointcloud)]
    else
      newNodes = (await Promise.all(prevNodes.map(async node =>
        Object.values(await waitForDef(() => node.children))
      ))).flat();
    return newNodes.filter(node => nodeFilterMethod(nodeFilterPrecomputedValues, node, pointcloud));
  }

  // Open new depths of the pointcloud until at least N points have been obtained.
  export async function getNPointcloudPoints(N, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues, pointFilterMethod, pointFilterPrecomputedValues) {
    let nodes = [];
    let points = [];
    let depth = 0;
    while(points.length < N) {
      // Get the next depth of nodes.
      nodes = await getNextDepthOfNodes(depth, nodes, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues);
      depth++;

      // If all nodes have been exhausted, quit.
      if(nodes.length == 0)
        break;
      
      // Add the points from these nodes.
      points = points.concat(await getPointsFromNodes(nodes,pointcloud,pointFilterMethod,pointFilterPrecomputedValues));
    }
    return points;
  }

  // Returns an mlmatrix containing the points, taken from the given nodes, filtered according to pointFilterMethod.
  export async function getPointsMatrixFromNodes(nodes, pointcloud, pointFilterMethod, pointFilterPrecomputedValues) {
    let pointcloudTransformation =  
    new Matrix4().makeTranslation(pointcloud.position.x,pointcloud.position.y,pointcloud.position.z).multiply(
      new Matrix4().makeScale(pointcloud.scale.x,pointcloud.scale.y,pointcloud.scale.z).multiply(
        new Matrix4().makeRotationFromEuler(pointcloud.rotation)
    ))
    return new Mlmatrix(
      // Maps nodes to an array of promises for their points, then await to get an array of every array of points. Then flatten and filter to get an array of array-points.
      (await Promise.all(nodes.map(async node => {
        // Extract point data from the node.
        loadNode(node);
        let position = await waitForDef(()=>node.geometry?.attributes?.position);
        // Each point is a row for now, because it make things faster. But it will be converted to columns later.
        let pointsMatrix = Mlmatrix.from1DArray(node.numPoints,3,position.array)
          // Add a column of ones for the 4D transformation matrix.
          .addColumn((new Array(node.numPoints)).fill(1));

        /* TEST CODE: Tests node transformation. Both of these should result in the same values. Currently passing. * /
          console.log(new Vector3(1,4,3.7).applyMatrix4(new Matrix4().multiplyMatrices(
            pointcloudMatrix, 
            new Matrix4().makeTranslation(node.boundingBox.min.x,node.boundingBox.min.y,node.boundingBox.min.z)
          )));
          
          console.log(new Vector3(1,4,3.7).add(node.boundingBox.min).applyEuler(pointcloud.rotation).multiply(pointcloud.scale).add(pointcloud.position));
        /**/

        // The transformation matrix to apply to the points of this node. 
        let nodeTransformation = matrix4ToMlmatrix(
          new Matrix4().multiplyMatrices(
            pointcloudTransformation, 
            new Matrix4().makeTranslation(node.boundingBox.min.x,node.boundingBox.min.y,node.boundingBox.min.z)
        ));
        // Transpose the transformation and multiply it on the right instead of left, because pointsMatrix is transposed.
        let transformedPointsMatrix = pointsMatrix.mmul(nodeTransformation.transpose());
        console.log("transformed the points according to the node and pointcloud.")
        // Filter points according to pointFilterMethod.
        let filteredPointsArray = transformedPointsMatrix.data.filter(point => pointFilterMethod(pointFilterPrecomputedValues, new Vector3(...point)));
        console.log("filtered points")
        return filteredPointsArray;
      })))
      // Flatten to get a single array of points.
      .flat()
    ).transpose();
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