import { OctreeGeometryNode } from "../src/modules/loader/2.0/OctreeGeometry";
import { BufferGeometry, BufferAttribute, Euler, Vector3, Matrix4 } from "../libs/three.js/build/three.module";
import { Matrix as Mlmatrix } from 'ml-matrix';
import { waitForDef, matrix4ToMlmatrix, mlmatrixConcat, arrayRandomSubset } from "./utils"

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
    
    // Once the original bounding box is transformed (including rotation,scale,position) then widenedBox will be a new axis-aligned box surrounding the old box
    // with extra room = maxDistance, so any qualifying points must be inside it.
    // Nodes all have same rotation and scale, so the relationship between position and widenedBox could be pre-comupted, but it is not very expensive.
    let projs = getBoxProjections([new Vector3(1,0,0), new Vector3(0,1,0), new Vector3(0,0,1)], node, pointcloud);
    let widenedBoxMin = {};
    let widenedBoxMax = {};
    for(let i = 0; i < 3; i++) {
      let dim = ["x","y","z"][i];
      widenedBoxMin[dim] = projs[i][0] - Math.sqrt(squaredMaxDistance);
      widenedBoxMax[dim] = projs[i][1] + Math.sqrt(squaredMaxDistance);
    }

    // Greatly reduce the size of the targetPoints to search by selecting only those in widenedBox.
    let candidatePoints = targetKdTree.subsection(widenedBoxMin, widenedBoxMax);

    // Apply scale to the bounding box, and reverse position and rotation to the targetPoints, because it's easier and equivalent to just applying all to the bounding box.
    // Rotating the bounding box messes up the method of determining if the point is inside.
    let min = node.boundingBox.min.clone().multiply(pointcloud.scale);
    let max = node.boundingBox.max.clone().multiply(pointcloud.scale);
    let reverseOrder = pointcloud.rotation.order[2] + pointcloud.rotation.order[1] + pointcloud.rotation.order[0]; // String reversal of "XYZ" or similar.
    let reverseRotation = new Euler(-pointcloud.rotation.x, -pointcloud.rotation.y, -pointcloud.rotation.z, reverseOrder);

    // See if any point is in range.
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
  // Projections may be stretched by normal's length. Best to have unit length if that matters.
  function getBoxProjections(normals, boundingBoxHolder, transformationHolder) {
    // Transform all 8 corners in advance. This is because normals will contain triplets of orthogonal vectors,
    // which require 6 distinct corners, overlapping between triplets.
    // This formation makes the opposite of cornerPoints[P] to be cornerPoints[7-P]
    let cornerPoints = Array(8);
    for(let i = 0; i < 8; i++) {
      cornerPoints[i] = new Vector3(
        boundingBoxHolder.boundingBox[i % 2 ? "max" : "min"].x,
        boundingBoxHolder.boundingBox[Math.floor(i/2) % 2 ? "max" : "min"].y,
        boundingBoxHolder.boundingBox[Math.floor(i/4) % 2 ? "max" : "min"].z
      ).applyEuler(transformationHolder.rotation).multiply(transformationHolder.scale).add(transformationHolder.position);
    }

    // The plan is to rotate unit vectors along with the box, then see which unit vectors point along with or against the normal.
    // Follow all the unit vectors pointing with the normal to find the max corner. Follow the ones pointing against to find the min corner.
    // (unless scale is negative, which would swap them)
    return normals.map(normal => {
      let minProjIdx = 0;
      if(new Vector3(1,0,0).applyEuler(transformationHolder.rotation).dot(normal) > 0)
        minProjIdx += 1;
      if(new Vector3(0,1,0).applyEuler(transformationHolder.rotation).dot(normal) > 0)
        minProjIdx += 2;
      if(new Vector3(0,0,1).applyEuler(transformationHolder.rotation).dot(normal) > 0)
        minProjIdx += 4;
      
      let minProj = cornerPoints[minProjIdx].dot(normal);
      let maxProj = cornerPoints[7-minProjIdx].dot(normal);

      if(transformationHolder.scale > 0)
        return [minProj, maxProj];
      else
        return [maxProj, minProj];
    });
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
        let position = await waitForDef(()=>node.position);
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

  // This method returns N points from the pointcloud.
  // It fully exhausts as many depths of the pointcloud as it can without exceeding N points.
  // Then it will randomly sample from the first unexhausted depth to fill N, if possible.
  // 
  // Fully exhausted depths do not need to be searched next time because their points will be stored in exhaustedPointsMatrix
  // which should be passed back into the function on the next call. On the first call it should be new Mlmatrix(0,0).
  // The first unexhausted depth will be stored in unexhaustedNodes. On the first call it should be getRootNode(pointcloud).
  //
  // Returns [N points Mlmatrix, exhaustedPointsMatrix, unexhaustedNodes]
  export async function getNPointsMatrix(N, exhaustedPointsMatrix, unexhaustedNodes, 
  pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues, pointFilterMethod, pointFilterPrecomputedValues) {

    // Subtract already found points from target N.
    N = Math.floor(N) - exhaustedPointsMatrix.columns;
    // Finish early if N is already reached.
    if(N <= 0)
      return [new Mlmatrix(0,0), exhaustedPointsMatrix, unexhaustedNodes];

    let unexhaustedPointsMatrix;
    // This loop fully exhausts depths of the pointcloud.
    while(true) {
      // Update unexhaustedPointsMatrix to the satisfactory points from this depth.
      unexhaustedPointsMatrix = await getPointsMatrixFromNodes(unexhaustedNodes, pointcloud, pointFilterMethod, pointFilterPrecomputedValues);

      // If there are no more depths, quit exhausting depths.
      if(unexhaustedNodes.length == 0)
        break;

      // If these points would overfill N, quit exhausting depths.
      if(unexhaustedPointsMatrix.columns > N)
        break;

      // Exhaust this depth, taking all of its points.
      exhaustedPointsMatrix = mlmatrixConcat(exhaustedPointsMatrix, unexhaustedPointsMatrix);
      N -= unexhaustedPointsMatrix.columns;
      // Descend to the next depth.
      unexhaustedNodes = await getNextDepthOfNodes(unexhaustedNodes, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues);
    }
   
    // Sample points from the next depth of the pointcloud, if it can and if it needs to.
    let sampledPointsMatrix;
    if(unexhaustedNodes.length == 0)
      sampledPointsMatrix = new Mlmatrix(0,0);
    else {
      sampledPointsMatrix = new Mlmatrix(unexhaustedPointsMatrix.rows, N);
      // Sample N distinct points from from unexhaustedPointsMatrix and put them in sampledPointsMatrix.
      let sampledIndexes = arrayRandomSubset(N,unexhaustedPointsMatrix.columns)
      for(let i = 0; i < N; i++)
        for(let dim = 0; dim < unexhaustedPointsMatrix.rows; dim++)
        sampledPointsMatrix.data[dim][i] = unexhaustedPointsMatrix.data[dim][sampledIndexes[i]];
    }
    return [sampledPointsMatrix, exhaustedPointsMatrix, unexhaustedNodes];
  }

  // If depth == 0, use the pointcloud's root. Else, take the children of the previous array of nodes.
  // Filter according to nodeFilterMethod, and load so they can be accessed.
  // Precomputed values are values that are the same for all nodes/points and are expensive to compute every time. 
  // It is better to compute them just once and pass them into the function.
  export async function getNextDepthOfNodes(prevNodes, pointcloud, nodeFilterMethod, nodeFilterPrecomputedValues, depth = 1) {
    let newNodes = [];
    if(depth == 0)
      newNodes = [await getRootNode(pointcloud)]
    else
      newNodes = (await Promise.all(prevNodes.map(async node =>
        Object.values(await waitForDef(() => node.children))
      ))).flat();
    return newNodes.filter(node => nodeFilterMethod(nodeFilterPrecomputedValues, node, pointcloud));
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
    let retryTimes = 10;
    for(let i = 0; i < retryTimes; i++) {
      console.log("trying " + node.name)
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
          let buffers = e.data.attributeBuffers;

          Potree.workerPool.returnWorker(workerPath, worker);
          
          node.position = new BufferAttribute(new Float32Array(buffers["position"].buffer), 3);
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

        return;
      }catch(e){
        console.log(`failed to load ${node.name}`);
        console.log(e);
        console.log(`trying again!`);
      }
    }
  }