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

import {Matrix} from "ml-matrix";
import { getDistributedPlanePoints, getRandomPlanePoints, getVertexPoints } from "./ifcAccess";
import { getPointsFromPointcloudUsingClipVolume, getPointsFromPointcloudUsingMaxDistance } from "./potreeAccess";
import { getSimilarityTransformation } from "./umeyama";

import {Points, PointsMaterial, BufferGeometry, Matrix4, Vector3} from "three";
import { mlmatrixToMatrix4, array2DToMlmatrix, mlmatrixConcat, transposeArray2D } from "./utils";
import { KDTree } from "./kdtree";

// Tweakable. I am not sure the best parameters to set for these.
function getOverallParameters() {
    return {
        // Min and max (inclusive) depths to search in the pointcloud's octree. There is one set of iterations per depth. 
        // Depth 0 is the root of the tree, and depths greater than the tree's depth just take the full tree.
        minDepth: 0,
        maxDepth: 5,
        // At shallow depths, sometimes no or very few points will be selected from the source pointcloud, and the transformation will be inaccurate.
        // This parameter specifies a minimum number of found source points. If it is not met, the algorithm will move to the next depth.
        minSourcePoints: 100
    };
}

// Tweakable. I am not sure the best parameters to set for these.
function getDepthParameters(depth,pointcloud,targetModel,parameters) {
    // If change in fitness and RMSE between iterations is smaller than these thresholds, assume it converged, and increase the depth.
    parameters.convergenceFitness = 0.01;
    parameters.convergenceRMSE = 0.01;
    // If no convergence is reached, increase the depth after this many iterations.
    parameters.numIterations = 2*2**(parameters.maxDepth-depth+1);
    // The algorithm assumes the user roughly aligned the pointcloud on the IFC model. After this alignment, points in the source pointcloud that are
    // farther than this distance from any points in the target IFC model are considered irrelevant (this distance is measured before ICP's transformations)
    parameters.maxSampleDistance = Math.max(...(["x","y","z"].map(dim=>targetModel.geometry.boundingBox.max[dim] - targetModel.geometry.boundingBox.min[dim])))
        * 0.05;
    // The max range to search for neighboring points for matchups (this distance is measured after ICP's transformations)
    parameters.maxNeighborDistance = Math.max(...(["x","y","z"].map(dim=>targetModel.geometry.boundingBox.max[dim] - targetModel.geometry.boundingBox.min[dim])))
    * 0.05;//3*pointcloud.pcoGeometry.spacing/2**depth;
    // How many points to sample from the IFC mesh.
    parameters.ifcSampleCount = 10000*1.1**depth;
}

// Functionality for adding higher weight to source points that meet certain criteria. Currently unused.
function weightEvaluation(sourcePoint) {
    return 1;
}

// The main function. 
export async function MultiScaleICP(/*targetFilePath,*/ targetModel, sourcePointcloud, initialEstimate, clipVolume) {
    let result = {transformation:new Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]), scale:1, correspondences:[], fitness:0, inlier_rmse:0};

    let overallTransformation = new Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]);

    let parameters = getOverallParameters();

    let sourcePoints = [];
    for(let depth = parameters.minDepth; depth <= parameters.maxDepth; depth++) {
        getDepthParameters(depth,sourcePointcloud,targetModel,parameters);

        // If you switch targetPoints functions, change convertPointsArrayToMatrix to match.
        // Take points from vertices of IFC file
     // let targetPoints = await getVertexPoints(targetFilePath);
        // Sample points from the planes of the mesh proudced by IFCjs
        console.log("Obtaining points from the IFC model.")
        let targetPoints = getDistributedPlanePoints(targetModel,parameters.ifcSampleCount).map(point=>new Vector3(...point))
        let targetKdTree = new KDTree(targetPoints);
        console.log(targetPoints.length + " IFC points obtained")

        // Obtain points from the pointcloud
        console.log("Obtaining points from the pointcloud.")
        sourcePoints = sourcePoints.concat(await getPointsFromPointcloudUsingMaxDistance(parameters.maxSampleDistance**2,targetKdTree,depth,sourcePointcloud))
     // sourcePoints = sourcePoints.concat(await getPointsFromPointcloudUsingClipVolume(clipVolume,depth,sourcePointcloud));
        console.log(sourcePoints.length + " pointcloud points obtained.")
        // Weights are applied during the find-optimal-transformation stage, not the choose-neighbor-points stage.
        let weights = sourcePoints.map(sourcePoint=>weightEvaluation(sourcePoint))

        renderPoints(sourcePoints, "red");
        renderPoints(targetPoints, "green");
        if(sourcePoints.length <= 0 || sourcePoints.length < parameters.minSourcePoints) {
            console.log("Too few source points were in range of the target at this depth. Skipping this depth.")
            continue;
        }
        DoSingleScaleICPIterations(sourcePoints,targetKdTree,weights,parameters,result);
        if(isNaN(result.scale) || result.scale === Infinity || result.scale === -Infinity || result.scale === 0) {
            console.log("Scale is invalid. Aborting.");
            return new Matrix4();
        }
        if(result.inlier_rmse == 0)
            break;
        overallTransformation = result.transformation.mmul(overallTransformation);
        sourcePointcloud.applyMatrix4(mlmatrixToThreematrix(result.transformation));
        console.log(result.transformation);
        result.transformation = new Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]);
    }
    renderPoints(sourcePoints, "blue", overallTransformation)
    return;
}

function DoSingleScaleICPIterations(sourcePoints,targetKdTree,weights,parameters,result) {

    // Turn sourcePoints into a matrix that can be transformed, and apply result.transformation to it.
    console.log("Convert points array to matrix")
    let sourceMatrix = result.transformation.mmul(convertPointsArrayToMatrix(sourcePoints))

    // Choose neighbor points and put them in result.correspondences.
    // Also put the percent of sourcePoints matched and the RMSE of the matched pairs in the result object.
    console.log("Find initial neighbor matchups")
    UpdateRegistrationResult(sourceMatrix,targetKdTree,parameters.maxNeighborDistance,result)

    console.log("Before: ","RMSE: " + result.inlier_rmse, "% sourcepoints matched: " + result.fitness, "scale transformation: " + result.scale)
    for(let i = 0; i < parameters.numIterations; i++) {
        prev_fitness = result.fitness
        prev_inlier_rmse = result.inlier_rmse
        
        // Possible causes: perfect fit, none of the source points found neighbors, or sourcePoints shrank to 0.
        // In all cases, further progress is impossible.
        if(result.inlier_rmse == 0) {
            console.log("RMSE is 0. Proceeding to next depth.")
            break
        }

        // Compute the optimal transformation for the given matchings of points. 
        console.log("Compute best transformation for this matchup.")
        let [tempTransform,tempScale] = ComputeTransformationPointToPoint(result.correspondences,weights)

        // If ICP goes wrong and the sourcePoints set contracts to 0 or expands to Infinity, quit before the special values cause problems.
        result.scale *= tempScale
        if(isNaN(result.scale) || result.scale === Infinity || result.scale === -Infinity || result.scale === 0) {
            console.log("Scale transformation became infinite or zero. Aborting.")
          break
        }
        
        // Apply the new transformation to sourceMatrix, and track it in result.transformation. Also track result.scale.
        result.transformation = tempTransform.mmul(result.transformation)
        sourceMatrix = tempTransform.mmul(sourceMatrix)

        // Choose new neighbor points and put them in result.correspondences.
        // Also update result.inlier_rmse and result.fitness.
        console.log("Find new neighbor matchups")
        UpdateRegistrationResult(sourceMatrix,targetKdTree,parameters.maxNeighborDistance,result)
        
        console.log("RMSE " + result.inlier_rmse)

        // Check convergence thresholds to determine if the transformation has converged and stopped improving.
        if(i != 0 && Math.abs(prev_fitness - result.fitness) < parameters.convergenceFitness 
            && Math.abs(prev_inlier_rmse - result.inlier_rmse) < parameters.convergenceRMSE) {
                console.log("Iterations have converged. Proceeding to next depth.")
                break
        }
    }
    console.log("After: ","RMSE: " + result.inlier_rmse, "% sourcepoints matched: " + result.fitness, "scale transformation: " + result.scale)
}

// Updates the result object.
// result.correspondences: neighbors for each source point in the target points, if it has a neighbor within maxNeighborDistance.
// result.fitness: % of the source points that have neighbors within maxNeighborDistance
// result.inlier_rmse: root mean squared distance between neighbors.
function UpdateRegistrationResult(sourceMatrix,targetKdTree,maxNeighborDistance,result) {
    let squaredError = 0;
    let sourceCount = sourceMatrix.columns;

    result.correspondences = Array(sourceMatrix.columns);
    for(let i = 0; i < sourceCount; i++) {
        // Take a single column of the matrix, cutting off the row of 1s, to get a single point.
        let sourcePoint = new Vector3(...sourceMatrix.getColumn(i));
        // Find the nearest neighbor in range, if there is one.
        result.correspondences[i] = [sourcePoint, targetKdTree.search(sourcePoint, maxNeighborDistance**2)];
    }
    // Filter out the failures.
    result.correspondences = result.correspondences.filter(correspondence => (correspondence[1][0] !== null));
    
    // Compute fitness
    let numCorrespondences = result.correspondences.length;
    result.fitness = numCorrespondences/sourceMatrix.columns;
    // Compute RMSE and discard individual point distances.
    for(let i = 0; i < result.correspondences.length; i++) {
        squaredError += result.correspondences[i][1][1];
        result.correspondences[i][1] = result.correspondences[i][1][0];
    }
    result.inlier_rmse = (numCorrespondences == 0 ? 0 : Math.sqrt(squaredError/numCorrespondences))
  }

function convertPointsArrayToMatrix(points) {
    let numPoints = points.length;
    // See MultiScaleICP function for switching between vertex points and plane points.
    // let dims = points[0].Coordinates.length; // For vertex points from the IFC data; the Test function doesn't work with this
    let dims = points[0].length; // For plane points sampled from the mesh generated by IFCJS
    let matrix = new Matrix(dims,numPoints);
    for(let c = 0; c < numPoints; c++)
        // matrix.setColumn(c,points[c].Coordinates) // For vertex points
        matrix.setColumn(c,points[c]) // For plane points
    // This bottom row filled with 1s is needed for 4x4 transformation matrices to be applicable to the array.
    matrix.addRow(matrix.rows,Matrix.ones(1,numPoints));
    return matrix;
}

function ComputeTransformationPointToPoint(correspondences,weights) {
    let numPoints = correspondences.length
    let source_mat = new Matrix(3,numPoints);
    let target_mat = new Matrix(3,numPoints);

    for(let i = 0; i < numPoints; i++) {
        source_mat.setColumn(i, correspondences[i][0].toArray());
        target_mat.setColumn(i, correspondences[i][1].toArray());
    }

    return getSimilarityTransformation(source_mat,target_mat,weights);
}

function mlmatrixToThreematrix(mlmatrix) {
    return (new Matrix4()).set(...(mlmatrix.to1DArray()));
}

// Point to plane. Not fully implemented.

/*
function ComputeTransformationPointToPlane(source, target, correspondences) {
    if(correspondences.empty() || !target.HasNormals())
        return Matrix.Identity
    
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
    return is_success ? extrinsic : Matrix.Identity
}

SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ,JTr) {
    let (solution_exist, x) = SolveLinearSystemPSD(JTJ, -JTr)

    if(solution_exist) {
        let extrinsic = TransformVector6dToMatrix4d(x)
        return (solution_exist, move(extrinsic))
    }
    return (false, Matrix.Identity)
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

    let correspondences = [0,1,2,3,4,5,6].map(i => [new Vector3(...source[i]),new Vector3(...target[i])])

    let result = ComputeTransformationPointToPoint(correspondences,weights);

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

function renderPoints(points, color, transformation) {
    let pointsObject = new Points(
        new BufferGeometry().setFromPoints(points), 
        new PointsMaterial({color:color, size: 1, sizeAttenuation: false})
    );

    if(transformation)
        pointsObject.applyMatrix4(mlmatrixToThreematrix(transformation));

    viewer.scene.scene.add(pointsObject);
}