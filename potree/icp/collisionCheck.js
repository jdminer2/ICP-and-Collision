/**
ICP with scale algorithm, which is a modified version from the ICP function 
available at https://github.com/isl-org/Open3D.

Their License:
The MIT License (MIT)

Open3D: www.open3d.org
Copyright (c) 2018-2023 www.open3d.org

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

SPDX-License-Identifier: MIT

Their Citation:
"
Please cite our work if you use Open3D.
@article{Zhou2018,
    author    = {Qian-Yi Zhou and Jaesik Park and Vladlen Koltun},
    title     = {{Open3D}: {A} Modern Library for {3D} Data Processing},
    journal   = {arXiv:1801.09847},
    year      = {2018},
}
"
 */

import { Matrix as Mlmatrix, inverse } from "ml-matrix";
import { getRandomSurfacePoints, getDistributedSurfacePoints } from "./ifcAccess";
import { getNPointsMatrix, getRootNode } from "./potreeAccess";
import { maxDistanceNodeFilterMethod, maxDistancePointFilterMethod } from "./potreeAccess";
import { clipVolumePrecomputedValues, clipVolumeNodeFilterMethod, clipVolumePointFilterMethod } from "./potreeAccess";
import { getSimilarityTransformation } from "./umeyama";

import {Points, PointsMaterial, BufferGeometry, Matrix4, Vector3} from "three";
import { mlmatrixToMatrix4, array2DToMlmatrix, mlmatrixConcat, transposeArray2D } from "./utils";
import { KDTree } from "./kdtree";

function checkTrianglesCollide(triangleA,triangleB,transl,rot,scale) {
    Anorm = crossproduct(A2-A1,A3-A1);
    if(dot(A1-B1,Anorm) == 0)
        d=0;
    else if(dot(B2-B1,Anorm) == 0)
        d=-1;
    else
        d = dot(B2-B1,Anorm)/dot(A1-B1,Anorm);

    if(0<=d<=1)
        P=B1+d*(B2-B1);
}



function AtoBVector(u,v) {
    return new Vector3().subVectors(v,u);
}

// Analyzes a triangle and returns [numDimensions, the triangle point opposite the longest side, vector1,vector2,vector3] where vector1,vector2,vector3 are all perpendicular.
// If the triangle is a 0-dimensional point, vector1,vector2,vector3 are just the three unit vectors.
// If the triangle is a 1-dimensional line segment, vector1 is the middle length side, and the others are perpendicular unit vectors.
// If the triangle is a 2-dimensional triangle, vector1 is the middle length side, vector2 is the smallest length side, and vector3 is a unit vector perpendicular to the triangle plane.
function dimsOfTriangle(trianglePoint1,trianglePoint2,trianglePoint3) {
    // Let v0 be the smallest side of the triangle, and v1 be the second smallest.
    let AB = AtoBVector(trianglePoint1,trianglePoint2);
    let AC = AtoBVector(trianglePoint1,trianglePoint3);
    let BC = AtoBVector(trianglePoint2,trianglePoint3);
    let vectors = [{vector:AB, lengthSq:AB.lengthSq(), idx:0},
                   {vector:AC, lengthSq:AC.lengthSq(), idx:1},
                   {vector:BC, lengthSq:BC.lengthSq(), idx:2}
                  ].sort((a,b)=>a.lengthSq-b.lengthSq);
    let v0 = vectors[0].vector;
    let v1 = vectors[1].vector;

    // If both vectors are near 0 length, this is a 0-dimensional shape.
    if(vectors[1].lengthSq < Math.pow(minDistanceToTouch,2))
        return [0,trianglePoint1,new Vector3(1,0,0),new Vector3(0,1,0),new Vector3(0,0,1)];

    let trianglePoint;
    // If largest side is AB
    if(vectors[2].idx == 0) {
        trianglePoint = trianglePoint3;
        // Make the vectors point out of trianglePoint
        v0.negate();
        v1.negate();
    }
    // If largest side is AC
    else if(vectors[2].idx == 1) {
        trianglePoint = trianglePoint2;
        // Make the vectors point out of trianglePoint
        if(vectors[0].idx == 0)
            v0.negate();
        else
            v1.negate();
    }
    // If largest side is BC
    else if(vectors[2].idx == 2)
        trianglePoint = trianglePoint1;
        // Vectors already point out of trianglePoint

    // Find a not-near-0 component of v1.
    let longestDim = "x";
    let longestDimLength = v1.x;
    if(v1.y > longestDimLength) {
        longestDim = "y";
        longestDimLength = v1.y;
    }
    if(v1.z > longestDimLength) {
        longestDim = "z"
        longestDimLength = v1.z
    }

    let numDimensions = 2;

    // If v0 is just some % of v1, triangle is 1-dimensional. Find its percentage of longestDim to check.
    let x = v0[longestDim]/longestDimLength;
    // This if statement isn't exactly right.
    if(distanceSq(scalarMult(x,v1),v0) < Math.pow(minDistanceToTouch,2)) {
        numDimensions = 1;
        // Replace v0 with a perpendicular vector to v1.
        if(longestDim == "x")
            v0 = new Vector3(v.y, -v.x, 0).normalize();
        else
            v0 = new Vector3(0, v.z, -v.y).normalize();
    }
    return [numDimensions,trianglePoint1,v1,v0,new Vector3().crossVectors(u,v).normalize()];
}

// Returns the point at which the line and plane intersect
function findLinePlaneCollision(planePoint,planeNorm,linePoint,lineVector) {
    let X = dotProduct(AtoBVector(linePoint,planePoint),planeNorm);
    if(X == 0)
        return linePoint.clone();
    let Y = dotProduct(lineVector,planeNorm);
    if(Y == 0)
        return null;
    return add(linePoint,scalarMult(Y/X,lineVector));
}

/*
point+norm=X
rotTrans(point)+rotTransedNorm=rotTrans(X)
rotTransedNorm=rotTrans(X)-rotTrans(point)
rotTransedNorm=RX+t-(Rp+t)
rotTransedNorm=R(X-point)=R(norm)
*/

/*
function findMovingPlaneCollision(planePoint,planeNorm,linePoint,lineVector,planeTrans,planeRotCenter,planeRotAxis,planeRotAngle) {
    forall 0<=t<=1:
        X = dotProduct(AtoBVector(linePoint,apply(planePoint,planeRotCenter,planeRotAxis,t*planeRotAngle)+t*planeTrans),apply(planeNorm,0,planeRotAxis,t*planeRotAngle));
        if(X == 0)
            return [t,linePoint.clone()];
        let Y = dotProduct(lineVector,apply(planeNorm,0,planeRotAxis,t*planeRotAngle));
        if(Y==0)
}

function findTriangleIntersectsPlane(planePoint,planeNormal,trianglePoint1,trianglePoint2,trianglePoint3) {
    let overs = [];
    let unders = [];
    let intersections = [];
    let minScore;
    [trianglePoint1,trianglePoint2,trianglePoint3].forEach((trianglePoint,i) => {
        // Proj vector from planePoint to trianglePoint onto normal's direction. Length of this is signed distance from plane to point.
        let score = dotProduct(AtoBVector(planePoint,trianglePoint),planeNormal)/planeNormal.length;
        let absScore = Math.abs(score);

        if(absScore < minDistanceToTouch)
            intersections.push(trianglePoint);
        else if(score > 0) {
            overs.push(trianglePoint);
            unders.forEach(underPoint => 
                intersections.push(findLinePlaneCollision(planePoint,planeNormal,underPoint,trianglePoint))
            );
        }
        else {
            unders.push(trianglePoint);
            overs.forEach(overPoint => 
                intersections.push(findLinePlaneCollision(planePoint,planeNormal,overPoint,trianglePoint))
            );
        }

        if (i == 0 || absScore < minScore)
            minScore = absScore;
    });
}

// https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
function findTriangleContainsCoplanarPoint(point,trianglePoint1,trianglePoint2,trianglePoint3) {
    let v0 = AtoBVector(trianglePoint1,trianglePoint2);
    let v1 = AtoBVector(trianglePoint1,trianglePoint3);
    let v2 = AtoBVector(trianglePoint1,point);

    let d00 = dotProduct(v0,v0);
    let d01 = dotProduct(v0,v1);
    let d11 = dotProduct(v1,v1);
    let d20 = dotProduct(v2,v0);
    let d21 = dotProduct(v2,v1);

    let denom = d00*d11-d01*d01;
    let v = (d11*d20-d01*d21)/denom;
    let w = (d00*d21-d01*d20)/denom;
    let u = 1-v-w;
}




function checkListsOverlap(trianglesA,trianglesB) {
    if(trianglesA.length == 0 || trianglesB.length == 0)
        return false;

    let triangleToPoints = (triangle,triangleIdx) => triangle.map(point => ({point, triangleIdx}));
    let pointsA = trianglesA.flatMap(triangleToPoints);
    let pointsB = trianglesB.flatMap(triangleToPoints);
    
    let pointSort = ["x","y","z"].map(dim => ((a,b)=>a.point[dim]-b.point[dim]));
    let sortedPointsAx = pointsA.toSorted(pointSort[0]);
    let sortedPointsAy = pointsA.toSorted(pointSort[1]);
    let sortedPointsAz = pointsA.toSorted(pointSort[2]);
    let sortedPointsBx = pointsB.toSorted(pointSort[0]);
    let sortedPointsBy = pointsB.toSorted(pointSort[1]);
    let sortedPointsBz = pointsB.toSorted(pointSort[2]);
    
    let minAIdx = {x:0,y:0,z:0};
    let minBIdx = {x:0,y:0,z:0};
    let maxAIdx = {x:sortedPointsAx.length - 1, y:sortedPointsAy.length - 1, z:sortedPointsAz.length - 1};
    let maxBIdx = {x:sortedPointsBx.length - 1, y:sortedPointsBy.length - 1, z:sortedPointsBz.length - 1};

    let keepA = trianglesA.map(_ => true);
    let keepB = trianglesB.map(_ => true);

    for(let i = 0; i < trianglesA.length; i++) {
        while(minAIdx.x <= maxAIdx.x && !keepA[minAIdx.x])
            minAIdx.x++;
        if(minAIdx.x > maxAIdx.x)
            return false;
        while(!keepA[maxAIdx.x])
            maxAIdx.x--;
        
        let triangle = trianglesA[i];
        if triangle.every(point => point.x > minAxIdxIdx)
    }

    min = sortedTrianglesAx[0].val;
    max = sortedTrianglesAx[sortedtrianglesAx.length - 1].val;
    for(let i = 0; i < sortedTrianglesBx.length; i++) {
        let point = sortedTrianglesBx[i];
        if(point.val > max)
            break;
        if(point.val < min)
            continue;
        keepB[point.index] = true;
    }
    trianglesBClone = trianglesBClone.filter(triangle => keepB[triangle.index]);


    keepA = trianglesA.map(_ => false);
    min = sortedPointsBy[0].val;
    max = sortedPointsBy[sortedPointsBy.length - 1].val;
    for(let i = 0; i < sortedPointsAy.length; i++) {
        let point = sortedPointsAy[i];
        if(point.val > max)
            break;
        if(point.val < min)
            continue;
        keepA[point.index] = true;
    }
    trianglesAClone = trianglesAClone.filter(triangle => keepA[triangle.index]);
    sortedPointsAy = sortedPointsAy.filter(point => keepA[point.index]);

    keepB = trianglesB.map(_ => false);
    min = sortedTrianglesAy[0].val;
    max = sortedTrianglesAy[sortedtrianglesAy.length - 1].val;
    for(let i = 0; i < sortedTrianglesBy.length; i++) {
        let point = sortedTrianglesBy[i];
        if(point.val > max)
            break;
        if(point.val < min)
            continue;
        keepB[point.index] = true;
    }
    trianglesBClone = trianglesBClone.filter(triangle => keepB[triangle.index]);


    keepA = trianglesA.map(_ => false);
    min = sortedPointsBz[0].val;
    max = sortedPointsBz[sortedPointsBz.length - 1].val;
    for(let i = 0; i < sortedPointsAz.length; i++) {
        let point = sortedPointsAz[i];
        if(point.val > max)
            break;
        if(point.val < min)
            continue;
        keepA[point.index] = true;
    }
    trianglesAClone = trianglesAClone.filter(triangle => keepA[triangle.index]);
    sortedPointsAz = sortedPointsAz.filter(point => keepA[point.index]);

    keepB = trianglesB.map(_ => false);
    min = sortedTrianglesAz[0].val;
    max = sortedTrianglesAz[sortedtrianglesAz.length - 1].val;
    for(let i = 0; i < sortedTrianglesBz.length; i++) {
        let point = sortedTrianglesBz[i];
        if(point.val > max)
            break;
        if(point.val < min)
            continue;
        keepB[point.index] = true;
    }
    trianglesBClone = trianglesBClone.filter(triangle => keepB[triangle.index]);

    sortedTrianglesAz = trianglesA.map((triangle,i) => [{val:triangle[0].z,index:i},{val:triangle[1].z,index:i},{val:triangle[2].z,index:i}]).sort((a,b)=>a.val-b.val);
    
    sortedTrianglesBz = trianglesB.map((triangle,i) => [{val:triangle[0].z,index:i},{val:triangle[1].z,index:i},{val:triangle[2].z,index:i}]).sort((a,b)=>a.val-b.val);


    trianglesA.forEach(triangle=>{
        ["x","y","z"].forEach(dimension => {
            let sortedPointCoords = [triangle[0][dimension],triangle[1][dimension],triangle[2][dimension]].sort((a,b)=>a-b);
            extrema["maxA" + dimension] = Math.max(extrema["maxA" + dimension], sortedPointCoords[2]);
            extrema["minA" + dimension] = Math.min(extrema["minA" + dimension], sortedPointCoords[0]);
        })
    });
    trianglesB.reduce(triangle=>{
        ["x","y","z"].forEach(dimension => {
            let sortedPointCoords = [triangle[0][dimension],triangle[1][dimension],triangle[2][dimension]].sort((a,b)=>a-b);
            extrema["maxB" + dimension] = Math.max(extrema["maxB" + dimension], sortedPointCoords[2]);
            extrema["minB" + dimension] = Math.min(extrema["minB" + dimension], sortedPointCoords[0]);
        })
    })
}


function checkTrianglesCollide(stationaryTriangle,transformingTriangle,positionChange,rotationChange,scaleChange) {
    
}




// Tweakable. I am not sure the best parameters to set for these.
function getOverallParameters() {
    return {
        // Min and max (inclusive) depths to search in the pointcloud's octree. There is one set of iterations per depth. 
        // Depth 0 is the root of the tree, and depths greater than the tree's depth just take the full tree.
        minDepth: 0,
        maxDepth: 3,
        // At shallow depths, sometimes no or very few points will be selected from the source pointcloud, and the transformation will be inaccurate.
        // This parameter specifies a minimum number of found source points. If it is not met, the algorithm will move to the next depth.
        minSamplePoints: 100,
        // If few points find neighbors, the transformation will be inaccurate.
        // This parameter specifies a minimum number of neighbor matchups.
        minSourceMatches: 50,
        // If many source points find only a few unique target points as neighbors, the transformation will be inaccurate.
        // This parameter specifies a minimum number of unique target point neighbors.
        minTargetMatches: 1,
        convergenceFitness: 0,
        convergenceRMSE: 0,
        numIterations: 0,
        maxSampleDistance: 0,
        maxNeighborDistance: 0,
        schematicSampleCount: 0,
        pointcloudSampleCount: 0,
        // This number must be at least 1. The scale can only be multiplied or divided by this much at each depth of iterations.
        // maxScalePerIteration: 1.1,
        // This number must be at least 1. The scale can only be multiplied or divided by this much overall.
        maxScaleChange: 100,
        // Seed for distributed point sampling from the schematic.
        schematicFirstSeed: 2000
    };
}

// Tweakable. I am not sure the best parameters to set for these.
function updateDepthParameters(depth,pointcloud,targetModel,parameters) {
    // If change in fitness and RMSE between iterations is smaller than these thresholds, assume it converged, and increase the depth.
    parameters.convergenceFitness = 0.01;
    parameters.convergenceRMSE = 0.01;
    // If no convergence is reached, increase the depth after this many iterations.
    parameters.numIterations = 2*2**(parameters.maxDepth-depth+1);
    // The algorithm assumes the user roughly aligned the pointcloud on the schematic. After this alignment, points in the source pointcloud that are
    // farther than this distance from any points in the target scehmatic are considered irrelevant (this distance is measured before ICP's transformations)
    parameters.maxSampleDistance = 0.05 * Math.max(...(["x","y","z"].map(dim=>targetModel.geometry.boundingBox.max[dim] - targetModel.geometry.boundingBox.min[dim])));
    // The max range to search for neighbors (this distance is measured after ICP's transformations)
    parameters.maxNeighborDistance = Infinity//0.05/2**depth * Math.max(...(["x","y","z"].map(dim=>targetModel.geometry.boundingBox.max[dim] - targetModel.geometry.boundingBox.min[dim])));//3*pointcloud.pcoGeometry.spacing/2**depth;
    // How many schematic points should be used at this depth.
    parameters.schematicSampleCount = 5000*1.1**depth;
    // How many pointcloud points should be used at this depth.
    parameters.pointcloudSampleCount = 5000*1.1**depth;
}
*/

// The main function. 
export async function multiScaleICP(/*targetFilePath,*/ targetModel, sourcePointcloud, clipVolume) {
    let filteringPrecomputedValues = clipVolumePrecomputedValues(clipVolume,sourcePointcloud);
    // let filteringPrecomputedValues = {squaredMaxDistance: parameters.maxSampleDistance ** 2, targetKdTree: null, 
    //   {matrix:sourcePointcloud.matrix.clone(), rotation:sourcePointcloud.rotation.clone(), scale:sourcePointcloud.scale.clone(), position:sourcePointcloud.position.clone()}}
    let overallTransformation = new Mlmatrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]);

    // Parameters for ICP.
    let parameters = getOverallParameters();

    // List of the blue points objects put on screen, which represent the pointcloud sampled points after transformation.
    let afterPointsObjects = [];

    let targetPoints = [];

    let exhaustedPointsMatrix = new Mlmatrix(0,0);
    let unexhaustedNodes = [await getRootNode(sourcePointcloud)];
    for(let depth = 0; depth <= parameters.maxDepth; depth++) {
        updateDepthParameters(depth,sourcePointcloud,targetModel,parameters);

        console.log("Obtaining points from the schematic mesh.");
        // Obtain target points and put them in a kdTree.
        let newTargetPoints = getDistributedSurfacePoints(parameters.schematicSampleCount - targetPoints.length,targetModel,parameters.schematicFirstSeed + targetPoints.length);
        targetPoints = targetPoints.concat(newTargetPoints);
        let targetKdTree = new KDTree(targetPoints);
        // Render points.
        renderPoints(newTargetPoints, "green");
        console.log(targetPoints.length + " schematic points");

        console.log("Obtaining points from the pointcloud.");
        // filteringPrecomputedValues.squaredMaxDistance = parameters.maxSampleDistance ** 2;
        // filteringPrecomputedValues.targetKdTree = targetKdTree;
        // Obtain the next depth of points from the pointcloud and add them to the transformable matrix.
        let newSourcePointsMatrix;
        [newSourcePointsMatrix, exhaustedPointsMatrix, unexhaustedNodes] = await getNPointsMatrix(parameters.pointcloudSampleCount, exhaustedPointsMatrix, unexhaustedNodes, 
            sourcePointcloud, clipVolumeNodeFilterMethod, clipVolumePointFilterMethod, filteringPrecomputedValues);
        let sourceMatrix = mlmatrixConcat(exhaustedPointsMatrix, newSourcePointsMatrix)
        // Render points.
        let beforePointsMatrix = inverse(overallTransformation).mmul(newSourcePointsMatrix);
        let beforePointsArray = [];
        for(let i = 0; i < beforePointsMatrix.columns; i++)
            beforePointsArray.push(new Vector3(beforePointsMatrix.data[0][i],beforePointsMatrix.data[1][i],beforePointsMatrix.data[2][i]));
        renderPoints(beforePointsArray, "red")
        let afterPointsArray = [];
        for(let i = 0; i < newSourcePointsMatrix.columns; i++)
            afterPointsArray.push(new Vector3(newSourcePointsMatrix.data[0][i],newSourcePointsMatrix.data[1][i],newSourcePointsMatrix.data[2][i]));
        afterPointsObjects.push(renderPoints(afterPointsArray, "blue"));
        console.log(sourceMatrix.columns + " pointcloud points.");

        // Skip the ICP iterations if there aren't enough points yet.
        if(depth < parameters.minDepth) {
            console.log("Not reached minDepth yet: proceeding to next depth.")
            continue;
        }
        if(sourceMatrix.columns < parameters.minSourcePoints) {
            console.log("Not enough pointcloud points: proceeding to next depth.");
            continue;
        }

        // This is currently unused.
        let weights = new Array(sourceMatrix.columns).fill(1);

        let scale = 1;
        // Perform one depth of ICP iterations.
        let result = singleICPDepth(sourceMatrix,targetKdTree,weights,parameters,scale);
        scale = result.scale;
        if(isNaN(scale) || scale === Infinity || scale === -Infinity || scale === 0) {
            console.log("Scale became extreme. Aborting with no useful results.");
            return new Matrix4();
        }

        // Apply result transformation.
        let resultMatrix4 = mlmatrixToMatrix4(result.transformation);
        overallTransformation = result.transformation.mmul(overallTransformation);
        exhaustedPointsMatrix = result.transformation.mmul(exhaustedPointsMatrix);
        sourcePointcloud.applyMatrix4(resultMatrix4);
        afterPointsObjects.forEach(afterPointsObject => afterPointsObject.applyMatrix4(resultMatrix4));

        console.log("Single depth transformation", result.transformation);
    }

    console.log("Overall transformation", overallTransformation);
    return overallTransformation;
}

function singleICPDepth(sourceMatrix,targetKdTree,weights,parameters,scale) {
    let result = {transformation:new Mlmatrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]), scale:scale};

    // Choose closest neighbors in maxNeighborDistance and put the pairings in neighborSet.neighborPairs.
    // Also put the percent of sourcePoints that had neighbors in range in neighborSet.fitness, and the RMSE of the pairings in neighborSet.RMSE.
    console.log("Finding initial neighbor pairings");
    let neighborSet = findNeighbors(sourceMatrix,targetKdTree,parameters.maxNeighborDistance);

    console.log("Before: ","RMSE: " + neighborSet.RMSE, "; % sourcepoints with neighbors: " + neighborSet.fitness);
    for(let i = 0; i < parameters.numIterations; i++) {
        prevFitness = neighborSet.fitness;
        prevRMSE = neighborSet.RMSE;

        // Check for enough points matched.
        if(neighborSet.neighborPairs.length < parameters.minSourceMatches) {
            console.log("Too few source matches. Proceeding to next depth.");
            break;
        }
        let targetMatches = [];
        for(let t1 = 0; t1 < neighborSet.neighborPairs.length; t1++) {
            let newPoint = neighborSet.neighborPairs[t1][1];
            let alreadyPresent = false;
            for(let t2 = 0; t2 < targetMatches.length; t2++)
                if(newPoint.x == targetMatches[t2].x && newPoint.y == targetMatches[t2].y && newPoint.z == targetMatches[t2].z) {
                  alreadyPresent = true;
                  break;
                }
            if(!alreadyPresent) {
                targetMatches.push(newPoint);
                if(targetMatches.length >= parameters.minTargetMatches)
                    break;
            }
        }
        if(targetMatches.length < parameters.minTargetMatches) {
            console.log("Too few unique target matches (" + targetMatches.length + "). Proceeding to next depth.");
            break;
        }

        // Compute the optimal transformation for the given pairings of points. 
        console.log("Computing best transformation from neighbor set.");
        let minScale = (1/parameters.maxScaleChange)/result.scale;
        let maxScale = parameters.maxScaleChange/result.scale;
        let [tempTransform,tempScale] = getBestTransformation(neighborSet.neighborPairs,weights,minScale,maxScale)

        // If sourcePoints set contracts to 0 or expands to Infinity, quit before math exceptions occur.
        result.scale *= tempScale;
        if(isNaN(result.scale) || result.scale === Infinity || result.scale === -Infinity || result.scale === 0) {
            console.log("Scale became extreme. Aborting.");
            break;
        }
        // Apply the new transformation to sourceMatrix, and track it in result.transformation.
        result.transformation = tempTransform.mmul(result.transformation);
        sourceMatrix = tempTransform.mmul(sourceMatrix);

        // Choose new closest neighbors.
        console.log("Finding new neighbor pairings");
        neighborSet = findNeighbors(sourceMatrix,targetKdTree,parameters.maxNeighborDistance);

        // Possible causes: perfect fit, extremely few source points found neighbors, or sourcePoints shrank to 0.
        // In any case, further progress at this depth is impossible.
        if(neighborSet.RMSE == 0) {
            console.log("RMSE is 0. Proceeding to next depth.");
            break;
        }

        // Check convergence thresholds to determine if the transformation has stopped improving.
        if(Math.abs(prevFitness - neighborSet.fitness) < parameters.convergenceFitness 
            && Math.abs(prevRMSE - neighborSet.RMSE) < parameters.convergenceRMSE) {
                console.log("Iterations have converged. Proceeding to next depth.");
                break;
        }
    }
    console.log("After: ","RMSE: " + neighborSet.RMSE, "; % sourcepoints with neighbors: " + neighborSet.fitness, "scale transformation: " + result.scale);
    return result;
}

// Pairs the transformed source points with their nearest neighbors in the target set, within maxNeighborDistance.
function findNeighbors(sourceMatrix,targetKDTree,maxNeighborDistance) {
    let sourceCount = sourceMatrix.columns;
    let totalSquaredDistance = 0;
    let neighborPairs = new Array(sourceCount);
    for(let i = 0; i < sourceCount; i++) {
        // Take a single column of the matrix, and turn it into a Vector3 point (ignoring the row of 1s automatically).
        let sourcePoint = new Vector3(...sourceMatrix.getColumn(i));
        // Get the closest neighbor in the target set.
        neighborPairs[i] = [sourcePoint, targetKDTree.search(sourcePoint, maxNeighborDistance**2)]
    }
    // Eliminate the pairs that didn't find a point in range, and get fitness.
    neighborPairs = neighborPairs.filter(correspondence => (correspondence[1][0] !== null));
    let fitness = neighborPairs.length/sourceCount;
    // Get RMSE, and cut out the distance of each pair because it is not needed.
    for(let i = 0; i < neighborPairs.length; i++) {
        totalSquaredDistance += neighborPairs[i][1][1];
        neighborPairs[i][1] = neighborPairs[i][1][0];
    }
    // Root Mean Squared Error between the source points and their neighbors.
    let RMSE = (neighborPairs.length == 0 ? 0 : Math.sqrt(totalSquaredDistance/neighborPairs.length))
    console.log("RMSE:" + RMSE);

    return {neighborPairs: neighborPairs, fitness:fitness, RMSE:RMSE}
  }

function getBestTransformation(neighborPairs,weights,minScale=0,maxScale=Infinity) {
    let numPoints = neighborPairs.length
    let source_mat = new Mlmatrix(3,numPoints);
    let target_mat = new Mlmatrix(3,numPoints);

    for(let i = 0; i < numPoints; i++) {
        source_mat.setColumn(i, neighborPairs[i][0].toArray());
        target_mat.setColumn(i, neighborPairs[i][1].toArray());
    }
    console.log(source_mat,target_mat)
    return getSimilarityTransformation(source_mat,target_mat,weights,minScale,maxScale);
}

// Point to plane. Not fully implemented.

/*
function ComputeTransformationPointToPlane(source, target, correspondences) {
    if(correspondences.empty() || !target.HasNormals())
        return Mlmatrix.Identity
    
    JTJ.setZero();
    JTr.setZero();
    for (let i = 0; i < correspondences.size(); i++) {
        vs = source.points[corres[i][0]]
        vt = target.points[corres[i][1]]
        nt = target.normals[corres[i][1]]

        J_r = vs.CrossProduct(nt).concat(nt)
        r = (vs-vt).Dot(nt)
        w = Weight(r)

        JTJ += J_r * w * J_r.transpose();
        JTr += J_r * w * r;
    }

    (is_success, extrinsic) = SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ,JTr)
    return is_success ? extrinsic : Mlmatrix.Identity
}

SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ,JTr) {
    let (solution_exist, x) = SolveLinearSystemPSD(JTJ, -JTr)

    if(solution_exist) {
        let extrinsic = TransformVector6dToMatrix4d(x)
        return (solution_exist, move(extrinsic))
    }
    return (false, Mlmatrix.Identity)
}

TransformVector6dToMatrix4d(input) {
    output.setIdentity()
    output.block<3,3>(0,0) = (AngleAxisd(input(2), UnitZ()) *
                              AngleAxisd(input(1), UnitY()) *
                              AngleAxisd(input(0), UnitX())).matrix()
    output.block<3,1>(0,3) = input.block<3,1>(3,0)
    return output
}
*/


/**
 * Tests the functionality responsible for optimal transforming a source point list to minimize its sum of distances to 
 * corresponding entries in the target point list.
 * All weights are 1 here, because giving different weights to different point pairs is currently an unused feature.
 * 
 * The example question and answer comes from https://zpl.fi/aligning-point-patterns-with-kabsch-umeyama-algorithm/
 */
export function testGetBestTransformation() {
    let source = [
        [232, 38, 0],
        [208, 32, 0],
        [181, 31, 0],
        [155, 45, 0],
        [142, 33, 0],
        [121, 59, 0],
        [139, 69, 0]
    ];
    let target = [
        [ 23, 178, 0],
        [ 66, 173, 0],
        [ 88, 187, 0],
        [119, 202, 0],
        [122, 229, 0],
        [170, 232, 0],
        [179, 199, 0]
    ];
    let weights = [1,1,1,1,1,1,1];

    let neighborPairs = [0,1,2,3,4,5,6].map(i => [new Vector3(...source[i]), new Vector3(...target[i])])

    let result = getBestTransformation(neighborPairs,weights);

    // getBestTransformation[0] should be multiplied with the first half of the neighborPairs array that was fed to it.
    // It will move the points to be very close to the second half.
    let answer = result[0].mmul(array2DToMlmatrix(transposeArray2D(source))).transpose().to2DArray();
    let correctAnswer = [
        [ 29.08878779, 152.36814188, 0],
        [ 52.37669337, 180.03008629, 0],
        [ 83.50028582, 204.33920503, 0],
        [126.28647155, 210.02515345, 0],
        [131.40664707, 235.37261559, 0],
        [178.54823113, 222.56285654, 0],
        [165.79288328, 195.30194121, 0]
    ];
    let scale = result[1];
    let correctScale = 1.46166131;

    let answerCorrect = true;
    // Assert same number of rows.
    if(correctAnswer.length != answer.length)
        answerCorrect = false;
    // For each row.
    for(let r = 0; r < correctAnswer.length && answerCorrect; r++) {
        // Assert same row length, except answer has an extra column of 1s.
        if(correctAnswer[r].length + 1 != answer[r].length)
            answerCorrect = false;
        // For each column.
        for(let c = 0; c <= correctAnswer[r].length && answerCorrect; c++) {
            // When not in the extra colummn of 1s
            if(c < correctAnswer[r].length) {
                // Assert values approximately equal
                if(Math.abs(correctAnswer[r][c] - answer[r][c]) > 0.0000001)
                    answerCorrect = false;
            }
            // When in the extra column of 1s
            else
                // Assert answer's values are 1.
                if(1 != answer[r][c])
                    answerCorrect = false;
        }
    }

    // Assert scale approximately equal.
    let scaleCorrect = Math.abs(correctScale - scale) < 0.0000001;

    console.log("Transformation", answerCorrect ? "Correct":"Incorrect; expected: " + JSON.stringify(correctAnswer) + " actual: " + JSON.stringify(answer))
    console.log("Scale", scaleCorrect ? "Correct":"Incorrect; expected: " + correctScale + " actual: " + scale )
}

function renderPoints(points, color) {
    let pointsObject = new Points(
        new BufferGeometry().setFromPoints(points), 
        new PointsMaterial({color:color, size: 1, sizeAttenuation: false})
    );

    viewer.scene.scene.add(pointsObject);
    return pointsObject;
}