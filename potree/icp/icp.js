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

import { Matrix as Mlmatrix } from "ml-matrix";
import { getDistributedPlanePoints, getRandomPlanePoints, getVertexPoints } from "./ifcAccess";
import { getRootNode, getNextDepthOfNodes, getPointsFromNodes, maxDistanceNodeFilterMethod, maxDistancePointFilterMethod, getNPoints } from "./potreeAccess";
import { getSimilarityTransformation } from "./umeyama";

import {Points, PointsMaterial, BufferGeometry, BufferAttribute, Matrix4, Vector3} from "three";
import { mlmatrixToMatrix4 } from "./utils";

// Tweakable. I am not sure the best parameters to set for these.
function getOverallParameters() {
    return {
        // Min and max (inclusive) depths to search in the pointcloud's octree. There is one set of iterations per depth. 
        // Depth 0 is the root of the tree, and depths greater than the tree's depth just take the full tree.
        minRound: 0,
        maxRound: 5,
        // At shallow depths, sometimes no or very few points will be selected from the source pointcloud, and the transformation will be inaccurate.
        // This parameter specifies a minimum number of found source points. If it is not met, the algorithm will move to the next depth.
        minSourcePoints: 100,
        convergenceFitness: 0,
        convergenceRMSE: 0,
        numIterations: 0,
        // The algorithm assumes the user roughly aligned the pointcloud on the IFC model. After this alignment, points in the source pointcloud that are
        // farther than this distance from any points in the target IFC model are considered irrelevant (this distance is measured before ICP's transformations)
        parameters.maxSampleDistance = Math.max(...(["x","y","z"].map(dim=>targetModel.geometry.boundingBox.max[dim] - targetModel.geometry.boundingBox.min[dim])))
        * 0.05;
        maxNeighborDistance: 0,
        ifcSampleCount: 0
    };
}

// Tweakable. I am not sure the best parameters to set for these.
function updateDepthParameters(depth,pointcloud,targetModel,parameters) {
    // If change in fitness and RMSE between iterations is smaller than these thresholds, assume it converged, and increase the depth.
    parameters.convergenceFitness = 0.01;
    parameters.convergenceRMSE = 0.01;
    // If no convergence is reached, increase the depth after this many iterations.
    parameters.numIterations = 2*2**(parameters.maxDepth-depth+1);
    // The max range to search for neighboring points for matchups (this distance is measured after ICP's transformations)
    parameters.maxNeighborDistance = Math.max(...(["x","y","z"].map(dim=>targetModel.geometry.boundingBox.max[dim] - targetModel.geometry.boundingBox.min[dim])))
    * 0.05/2**depth;//3*pointcloud.pcoGeometry.spacing/2**depth;
    // How many points to sample from the IFC mesh.
    parameters.ifcSampleCount = 10000*1.1**depth;
}

// The main function. 
export async function MultiScaleICP(/*targetFilePath,*/ targetModel, sourcePointcloud, initialEstimate, clipVolume) {
    let overallTransformation = new Mlmatrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]);

    // Parameters for ICP.
    let parameters = getOverallParameters();

    // Lists of points.
    console.log("Obtaining points from the IFC model.");
    let targetPoints = getDistributedPlanePoints(targetModel,parameters.ifcSampleCount);
    renderPoints(targetPoints, "green");
    console.log(targetPoints.length + " IFC points");

    console.log("Obtaining points from the pointcloud.");
    sourcePointsPrecomputedValues = {maxDistance: parameters.maxSampleDistance, targetPoints:targetPoints};
    let sourceMatrix = getPointcloudPointMatrix(parameters.pointcloudSampleCount, sourcePointcloud, maxDistanceNodeFilterMethod, sourcePointsPrecomputedValues, maxDistancePointFilterMethod, sourcePointsPrecomputedValues);
    renderedSourcePointsObjects.push(renderPoints(sourceMatrix.data.map(point => new Vector3(point.toSpliced(-1))), "red"));
    console.log(sourceMatrix.data.length + " pointcloud points.");

    // List of the red points objects put on screen, tracked so they can be rotated along with the source pointcloud.
    let renderedSourcePointsObjects = [];

    for(let round = parameters.minRound; round <= parameters.maxRound; depth++) {
        updateDepthParameters(depth,sourcePointcloud,targetModel,parameters);

        // This is currently unused.
        let weights = sourceMatrix.data.map(point=>1);

        let result = SingleDepthOfICPIterations(sourcePoints,targetPoints,weights,parameters);
        if(isNaN(result.scale) || result.scale === Infinity || result.scale === -Infinity || result.scale === 0) {
            console.log("Scale became extreme. Aborting.");
            return new Matrix4();
        }
        overallTransformation = result.transformation.mmul(overallTransformation);
        let transMatrix = mlmatrixToMatrix4(result.transformation);
        sourcePointcloud.applyMatrix4(transMatrix);
        renderedSourcePointsObjects.forEach(renderedPointsObject => renderedPointsObject.applyMatrix4(transMatrix));

        console.log(result.transformation);
    }
    console.log(overallTransformation);
}

function SingleDepthOfICPIterations(sourcePoints,targetPoints,weights,parameters) {
    let result = {transformation:new Mlmatrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]), scale:1};
    // Turn sourcePoints into a matrix that can be transformed.
    console.log("Converting points array to matrix");
    let sourceMatrix = convertPointsArrayToMatrix(sourcePoints);

    // Choose neighbor points and put them in result.correspondences.
    // Also put the percent of sourcePoints matched and the RMSE of the matched pairs in the result object.
    console.log("Finding initial neighbor matchups");
    let matchups = MatchNeighbors(sourceMatrix,targetPoints,parameters.maxNeighborDistance);

    console.log("Initial: ","RMSE: " + matchups.RMSE, "% sourcepoints matched: " + matchups.fitness);
    for(let i = 0; i < parameters.numIterations; i++) {
        prevFitness = matchups.fitness;
        prevRMSE = matchups.RMSE;

        // Compute the optimal transformation for the given matchings of points. 
        console.log("Computing best transformation for this matchup.");
        let [tempTransform,tempScale] = ComputeTransformationPointToPoint(matchups.pairs,weights)

        // If sourcePoints set contracts to 0 or expands to Infinity, quit before math exceptions occur.
        result.scale *= tempScale;
        if(isNaN(result.scale) || result.scale === Infinity || result.scale === -Infinity || result.scale === 0) {
            console.log("Scale became extreme. Aborting.");
            break;
        }
        // Apply the new transformation to sourceMatrix, and track it in result.transformation.
        result.transformation = tempTransform.mmul(result.transformation);
        sourceMatrix = tempTransform.mmul(sourceMatrix);

        // Choose new neighbor points.
        console.log("Finding new neighbor matchups");
        matchups = MatchNeighbors(sourceMatrix,targetPoints,parameters.maxNeighborDistance);

        // Possible causes: perfect fit, extremely few source points found neighbors, or sourcePoints shrank to 0.
        // In any case, further progress at this depth is impossible.
        if(matchups.RMSE == 0) {
            console.log("RMSE is 0. Proceeding to next depth.");
            break;
        }

        // Check convergence thresholds to determine if the transformation has stopped improving.
        if(Math.abs(prevFitness - matchups.fitness) < parameters.convergenceFitness 
            && Math.abs(prevRMSE - matchups.RMSE) < parameters.convergenceRMSE) {
                console.log("Iterations have converged. Proceeding to next depth.");
                break;
        }
    }
    console.log("After: ","RMSE: " + matchups.RMSE, "% sourcepoints matched: " + matchups.fitness, "scale transformation: " + result.scale);
    return result;
}

// Matches the transformed source points with their nearest neighbors in the target set, within maxNeighborDistance.
function MatchNeighbors(sourceMatrix,targetPoints,maxNeighborDistance) {
    let dims = sourceMatrix.rows - 1;
    let sourceCount = sourceMatrix.columns;
    let totalSquaredDistance = 0;
    let pairs = Array(sourceCount);
    for(let i = 0; i < sourceCount; i++) {
        // Take a single column of the matrix, cutting off the row of 1s, to get a single point.
        let sourcePoint = sourceMatrix.getColumn(i).slice(0,-1);
        let bestPoint = null;
        let bestDistance = maxNeighborDistance ** 2;
        for(let j = 0; j < targetPoints.length; j++) {
            let currentPoint = targetPoints[j];
            let currentDistance = 0;
            for(let dim = 0; dim < dims; dim++) {
                currentDistance += (sourcePoint[dim]-currentPoint[dim])**2;
                // Exit early to reduce computation, if it's already too far. This will happen most of the time.
                if(currentDistance > bestDistance)
                  break;
            }
            if(currentDistance < bestDistance) {
                bestDistance = currentDistance;
                bestPoint = currentPoint;
            }
        }
        // If no close neighbor was found, point will still be null.
        pairs[i] = [sourcePoint,bestPoint];
        if(bestPoint)
            totalSquaredDistance += bestDistance;
    }
    // Eliminate the null points
    pairs = pairs.filter(correspondence=>{return correspondence[1] !== null});
    // % of the sourcePoints that found neighbors in range.
    let fitness = pairs.length/sourceCount;
    // Root Mean Squared Error between the source points and their neighbors.
    let RMSE = (pairs.length == 0 ? 0 : Math.sqrt(totalSquaredDistance/pairs.length))
    console.log(RMSE);

    return {pairs: pairs, fitness:fitness, RMSE:RMSE}
  }

function convertPointsArrayToMatrix(points) {
    let numPoints = points.length;
    // See MultiScaleICP function for switching between vertex points and plane points.
    // let dims = points[0].Coordinates.length; // For vertex points from the IFC data; the Test function doesn't work with this
    let dims = points[0].length; // For plane points sampled from the mesh generated by IFCJS
    let matrix = new Mlmatrix(dims,numPoints);
    for(let c = 0; c < numPoints; c++)
        // matrix.setColumn(c,points[c].Coordinates) // For vertex points
        matrix.setColumn(c,points[c]) // For plane points
    // This bottom row filled with 1s is needed for 4x4 transformation matrices to be applicable to the array.
    matrix.addRow(matrix.rows,Mlmatrix.ones(1,numPoints));
    return matrix;
}

function ComputeTransformationPointToPoint(correspondences,weights) {
    let numPoints = correspondences.length
    let dims = correspondences[0][0].length
    let source_mat = new Mlmatrix(dims,numPoints);
    let target_mat = new Mlmatrix(dims,numPoints);

    for(let i = 0; i < numPoints; i++) {
        source_mat.setColumn(i, correspondences[i][0]);
        target_mat.setColumn(i, correspondences[i][1]);
    }

    return getSimilarityTransformation(source_mat,target_mat,weights);
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
 * All weights are 1 here, because giving different weights to different point correspondences is currently an unused feature.
 * 
 * The example question and answer comes from https://zpl.fi/aligning-point-patterns-with-kabsch-umeyama-algorithm/
 */
export function TestOptimalTransformationForIteration() {
    let source = [
        [232, 38],
        [208, 32],
        [181, 31],
        [155, 45],
        [142, 33],
        [121, 59],
        [139, 69]
    ];
    let target = [
        [ 23, 178],
        [ 66, 173],
        [ 88, 187],
        [119, 202],
        [122, 229],
        [170, 232],
        [179, 199]
    ];
    let weights = [1,1,1,1,1,1,1];

    let correspondences = [0,1,2,3,4,5,6].map(i => [source[i],target[i]])

    let result = ComputeTransformationPointToPoint(correspondences,weights);

    let answer = result[0].mmul(convertPointsArrayToMatrix(source)).transpose().to2DArray();
    let correctAnswer = [
        [ 29.08878779, 152.36814188],
        [ 52.37669337, 180.03008629],
        [ 83.50028582, 204.33920503],
        [126.28647155, 210.02515345],
        [131.40664707, 235.37261559],
        [178.54823113, 222.56285654],
        [165.79288328, 195.30194121]
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

function renderPoints(points, color, transformation) {
    let pointsObject = new Points(
        new BufferGeometry().setFromPoints(points), 
        new PointsMaterial({color:color, size: 1, sizeAttenuation: false})
    );

    if(transformation)
        pointsObject.applyMatrix4(mlmatrixToMatrix4(transformation));

    viewer.scene.scene.add(pointsObject);
}