import { OctreeGeometryNode } from "../src/modules/loader/2.0/OctreeGeometry";
import { BufferGeometry, BufferAttribute, Euler, Vector3, Matrix3, Matrix4 } from "../libs/three.js/build/three.module";
import { Matrix as Mlmatrix } from 'ml-matrix';
import { waitForDef, matrix4ToMlmatrix, mlmatrixConcat, arrayRandomSubset } from "./utils"

  // Precomputed values for the clipVolume point filtering strategy.
  // The transformation on sourcePointcloud when this method is called is the one that is used:
  //   To check for points that were originally inside clipVolume before transformation, call this method before transformation.
  //   To check for points that become inside clipVolume after some transformation, call this method after transformation.
  export function clipVolumePrecomputedValues(clipVolume,sourcePointcloud) {
    // According to the separating axis theorem, two convex polyhedra intersect if and only if the projections of the polyhedra overlap
    // for all normals of each polyhedra's faces. https://dyn4j.org/2010/01/sat/
    // A box has 3 normals: [1,0,0], [0,1,0], and [0,0,1], transformed by the box's rotation.
    // So we have 6 normals to work with. These will always be the same, so we can precompute them.
    let normals = [
      new Vector3(1,0,0).applyEuler(clipVolume.rotation),
      new Vector3(0,1,0).applyEuler(clipVolume.rotation),
      new Vector3(0,0,1).applyEuler(clipVolume.rotation),
      new Vector3(1,0,0).applyEuler(sourcePointcloud.rotation),
      new Vector3(0,1,0).applyEuler(sourcePointcloud.rotation),
      new Vector3(0,0,1).applyEuler(sourcePointcloud.rotation)
    ];

    // Projections of a point p onto a unit vector n = dotProduct(p,n)*n. It is all projecting onto the same direction, so we can ignore the *n.
    // The projections of boxes overlap if: check each corner and find the range [min,max] of dotProduct(corner of box1,n). Do the same for box2, and check if ranges overlap.
    // The projection of a box contains the projection of a point if: the above range contains dotProduct(point,n).

    // clipVolume will always be the same, so we can precompute its projection range.
    let clipVolumeProjections = getBoxProjections(normals, clipVolume, clipVolume);

    // node box will change depending on what nodes we explore. However, the transformation applied to it will always be the same.
    // Therefore we can save some work.

    // For a transformed vector: RSv+P
    // dotProduct(RSv+P,n)=dotProduct(v,transpose(RS)n)+dotProduct(P,n)
    // X will store transpose(RS)n (an array for each normal n)
    // Y will store clipVolumeProjections - dotProduct(P,n).

    // Therefore, box projections overlap if: the range [Y[0],Y[1]] overlaps [min,max] of dotProduct(untransformed corner of node box,X).
    // clipvolume projection contains point projection if: the range [Y[0],Y[1]] contains dotProduct(untransformed point,X).
    let X = normals.map(normal =>
      normal.applyMatrix3(new Matrix3().setFromMatrix4(sourcePointcloud.matrix).transpose())
    )

    let Y = normals.map((normal,i) => 
      [clipVolumeProjections[i][0]-sourcePointcloud.position.dot(normal),clipVolumeProjections[i][1]-sourcePointcloud.position.dot(normal)]
    );

    return {X,Y};
  }

  // Filtering method for nodes which only accepts nodes that may have points which were initially located inside the clipping box.
  export function clipVolumeNodeFilterMethod({X, Y}, node) {
    return Y.every((y,i) => {
      let min = 0;
      let max = 0;
      for(let dimension of ["x","y","z"]) {
        if(X[i][dimension] >= 0) {
          min += node.boundingBox.min[dimension] * X[i][dimension];
          max += node.boundingBox.max[dimension] * X[i][dimension];
        }
        else {
          max += node.boundingBox.min[dimension] * X[i][dimension];
          min += node.boundingBox.max[dimension] * X[i][dimension];
        }
      }

      return y[0] <= max && y[1] >= min;
    })
  }

  // Filtering method for points which only accepts points inside a box volume. Expects point to be a Vector3.
  export function clipVolumePointFilterMethod({X, Y}, point) {
    return Y.every((y,i) => {
      let val = point[0] * X[i].x + point[1] * X[i].y + point[2] * X[i].z;

      return y[0] <= val && y[1] >= val;
    });
  }

  
  // maxDistancePrecomputedValues should be {squaredMaxDistance, targetKdTree, initialPclTransformation} where targetPoints are in Vector3 form.
  // initialPclTransformation needs to have {matrix:Matrix4, rotation:Euler, scale:Vector3, position:Vector3}.
  
  // Filtering method for nodes which only accepts nodes that may have points within maxDistance of a target point.
  export function maxDistanceNodeFilterMethod({squaredMaxDistance, targetKdTree, initialPclTransformation}, node) {
    if(squaredMaxDistance < 0)
      return false;
    
    // Once the node bounding box is transformed according to initialPclTransformation, then widenedBox will be a new axis-aligned box surrounding the old box
    // with extra room = maxDistance, so any qualifying points must be inside it.
    // Transformation is always the same, so some more work could be pre-comupted, but it is not very helpful.
    let projs = getBoxProjections([new Vector3(1,0,0), new Vector3(0,1,0), new Vector3(0,0,1)], node, initialPclTransformation);
    let widenedBoxMin = {};
    let widenedBoxMax = {};
    for(let i = 0; i < 3; i++) {
      let dim = ["x","y","z"][i];
      widenedBoxMin[dim] = projs[i][0] - Math.sqrt(squaredMaxDistance);
      widenedBoxMax[dim] = projs[i][1] + Math.sqrt(squaredMaxDistance);
    }

    // Greatly reduce the size of the targetPoints to search by selecting only those in widenedBox.
    let candidatePoints = targetKdTree.subsection(widenedBoxMin, widenedBoxMax);

    // Instead of applying rotation, scale, and position to the node box, then checking for target points inside:
    // Apply scale to the bounding box, and reverse position and rotation to the targetPoints.
    // This keeps box axis-aligned so max and min corners work right. 
    // Rotation and scale are interchangeable, but position must be applied last. Or reverse-position must be applied first.
    let min = node.boundingBox.min.clone().multiply(initialPclTransformation.scale);
    let max = node.boundingBox.max.clone().multiply(initialPclTransformation.scale);
    // String reversal of "XYZ" or similar.
    let reverseOrder = initialPclTransformation.rotation.order[2] + initialPclTransformation.rotation.order[1] + initialPclTransformation.rotation.order[0]; 
    let reverseRotation = new Euler(-initialPclTransformation.rotation.x, -initialPclTransformation.rotation.y, -initialPclTransformation.rotation.z, reverseOrder);

    // See if any point is in range.
    return candidatePoints.some(targetPoint => {
      let newTargetPoint = targetPoint.clone().sub(initialPclTransformation.position).applyEuler(reverseRotation);
      let x = Math.max(0, newTargetPoint.x - max.x, min.x - newTargetPoint.x);
      let y = Math.max(0, newTargetPoint.y - max.y, min.y - newTargetPoint.y);
      let z = Math.max(0, newTargetPoint.z - max.z, min.z - newTargetPoint.z);
      return x**2 + y**2 + z**2 <= squaredMaxDistance;
    });
  }

  // Filtering method for points which only accepts points initially within maxDistance of a target point. Expects point to be an array and targetKdTree to be kdTree of Vector3 points.
  export function maxDistancePointFilterMethod({squaredMaxDistance, targetKdTree, initialPclTransformation}, point) {
    let vectorPoint = new Vector3(...point).applyMatrix4(initialPclTransformation.matrix);
    return targetKdTree.searchExistence(vectorPoint,squaredMaxDistance);
  }

// Apply transformation onto bounding box, then find the range [min,max] of dotProduct(corner of box,n), which is its projection range.
  // If normal isn't a unit vector, [min,max] will be stretched equal to normal's length.
  function getBoxProjections(normals, boundingBoxHolder, transformationHolder) {
    // Make cornerPoints array. The opposite corner of cornerPoints[P] will be cornerPoints[7-P].
    let cornerPoints = Array(8);
    for(let i = 0; i < 8; i++) {
      cornerPoints[i] = new Vector3(
        boundingBoxHolder.boundingBox[i % 2 ? "max" : "min"].x,
        boundingBoxHolder.boundingBox[Math.floor(i/2) % 2 ? "max" : "min"].y,
        boundingBoxHolder.boundingBox[Math.floor(i/4) % 2 ? "max" : "min"].z
      // Transform all 8 corners in advance.
      ).applyMatrix4(transformationHolder.matrix);
    }

    return normals.map(normal => {
      // Rotate unit vectors using rotation of the box. This means these vectors are parallel to the box's edges.
      // Find if the box's edges projected onto normal are positive or negative. Follow the negative edges to reach the min corner.
      let minProjIdx = 0;
      if(new Vector3(1,0,0).applyEuler(transformationHolder.rotation).dot(normal) < 0)
        minProjIdx += 1;
      if(new Vector3(0,1,0).applyEuler(transformationHolder.rotation).dot(normal) < 0)
        minProjIdx += 2;
      if(new Vector3(0,0,1).applyEuler(transformationHolder.rotation).dot(normal) < 0)
        minProjIdx += 4;

      // If scale is negative, it must be swapped. The opposite corner to idx is 7-idx.
      if(transformationHolder.scale < 0)
        minProjIdx = 7 - minProjIdx;

      return [
        cornerPoints[minProjIdx].dot(normal),
        cornerPoints[7-minProjIdx].dot(normal)
      ];
    });
  }

  // Returns an mlmatrix containing the points, taken from the given nodes, filtered according to pointFilterMethod.
  async function getPointsMatrixFromNodes(nodes, pointcloud, pointFilterMethod, pointFilterPrecomputedValues) {
    // Combine all nodes into a single 2D array where each row is a point, and there is an extra column of 1s for transformation purposes.
    let allPointData = (await Promise.all(nodes.map(async node => {
      // Extract 1D array of point data from the node.
      loadNode(node);
      let pointData1D = (await waitForDef(()=>node.position)).array;

      // Make it a 2D array of [x,y,z] points.
      let pointData = Array(node.numPoints);
      for(let i = 0; i < node.numPoints; i++)
        pointData[i] = pointData1D.slice(i*3, i*3+3);

      // Filter points according to pointFilterMethod.
      let filteredPointData = pointData.filter(point => pointFilterMethod(pointFilterPrecomputedValues, point));
      console.log("filtered points")
      
      // The transformation matrix to apply to the points of this node: combines pointcloud's transformation and node's position offset. 
      let nodeTransformation = matrix4ToMlmatrix(
        new Matrix4().multiplyMatrices(
          pointcloud.matrix, 
          new Matrix4().makeTranslation(node.boundingBox.min.x,node.boundingBox.min.y,node.boundingBox.min.z)
      ));

      // Convert 2D array to Mlmatrix, and add a column of 1s, to make a transformable points matrix
      // (but it is transposed because each point is a row instead of a column)
      let pointsMatrix = new Mlmatrix(filteredPointData).addColumn(new Array(filteredPointData.length).fill(1));
      
      // Apply the transformation to the points (in a transposed way) then extract the row data as a 2D array again to return.
      transformedPointData = pointsMatrix.mmul(nodeTransformation.transpose()).data;
      console.log("transformed the points according to the node and pointcloud.")

      return transformedPointData;
    // Combine each node's returned 2D array into a single 2D array
    }))).flat();
    // Convert to Mlmatrix and transpose so each point is a column.
    return new Mlmatrix(allPointData).transpose();
  }

  // This method returns N points from the pointcloud.
  // It fully exhausts as many depths of the pointcloud as it can without exceeding N points.
  // Then it will randomly sample from the first unexhausted depth to fill N, if possible.
  // 
  // Fully exhausted depths do not need to be searched next time because their points will be stored in exhaustedPointsMatrix
  // which should be passed back into the function on the next call. On the first call it should be new Mlmatrix(0,0).
  // The shallowest unexhausted depth will be stored in unexhaustedNodes. On the first call it should be getRootNode(pointcloud).
  //
  // The inputted transformation is not yet applied to the inputs, and should not be applied to the outputs.
  //
  // Returns [N points Mlmatrix, exhaustedPointsMatrix, unexhaustedNodes]
  export async function getNPointsMatrix(N, exhaustedPointsMatrix, unexhaustedNodes, pointcloud, 
    nodeFilterMethod, pointFilterMethod, filterPrecomputedValues) {
    // Subtract already found points from target N.
    N = Math.floor(N) - exhaustedPointsMatrix.columns;
    // Finish early if N is already reached.
    if(N <= 0)
      return [new Mlmatrix(0,0), exhaustedPointsMatrix, unexhaustedNodes];

    let unexhaustedPointsMatrix;
    // This loop fully exhausts depths of the pointcloud.
    while(true) {
      // Update unexhaustedPointsMatrix to the satisfactory points from this depth.
      unexhaustedPointsMatrix = await getPointsMatrixFromNodes(unexhaustedNodes, pointcloud, pointFilterMethod, filterPrecomputedValues);

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
      unexhaustedNodes = await getNextDepthOfNodes(unexhaustedNodes, nodeFilterMethod, filterPrecomputedValues);
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

  // Take the children of the previous array of nodes.
  // Filter according to nodeFilterMethod, and load so they can be accessed.
  // Precomputed values are values that are the same for all nodes/points and are expensive to compute every time. 
  // It is better to compute them just once and pass them into the function.
  async function getNextDepthOfNodes(prevNodes, nodeFilterMethod, nodeFilterPrecomputedValues) {
    // Get the children of each node.
    let newNodes = (await Promise.all(prevNodes.map(async node =>
      Object.values(await waitForDef(() => node.children))
    // Combine arrays of children into a single array of children.
    ))).flat();
    // Filter the children using nodeFilterMethod.
    return newNodes.filter(node => nodeFilterMethod(nodeFilterPrecomputedValues, node));
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