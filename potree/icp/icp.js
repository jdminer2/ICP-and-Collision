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
import { getPointsFromPointcloud } from "./potreeAccess";
import { getSimilarityTransformation } from "./umeyama";

// Tweakable
// At each depth:
    // convergenceFitness and convergenceRMSE: if change in fitness and rmse both get under their convergence thresholds,
    // then the transformation has basically stopped changing at this depth, so move to the next depth. 
    // numIterations: how many iterations can be done at this depth at most.
    // maxNeighborDistance: how far to search for nearby neighbors to points. this range will scale up with the source points
    // ifcPointCount: how many points to sample from the IFC mesh
    // There is no adjustable threshold for sampling from the other file. All points from the current depth are taken.
function getThresholds(depth,maxDepth,targetPointcloud) {
    return {
        convergenceFitness:0,
        convergenceRMSE:0,
        numIterations:2*2**(maxDepth-depth+1),
        maxNeighborDistance:1*targetPointcloud.pcoGeometry.spacing/2**depth,
        ifcPointCount:100*2**depth
    };
}

function weightEvaluation(sourceModel,sourcePoint) {
    return 1;
}

// The main function. 
export async function MultiScaleICP(sourceFilePath, sourceModel, targetPointcloud, initialEstimate, maxDepth) {
    let result = {transformation:initialEstimate, scale:1, correspondences:[], fitness:0, inlier_rmse:0};

    console.log(sourceModel)

    for(let depth = 0; depth <= maxDepth; depth++) {
        let thresholds = getThresholds(depth,maxDepth,targetPointcloud);
        // Must also change convertPointsArrayToMatrix if you enable this function
        // let sourcePoints = await getVertexPoints(sourceFilePath); // Take points from vertices of IFC file
        let sourcePoints = getDistributedPlanePoints(sourceModel,thresholds.ifcPointCount); // Sample points from the planes of the mesh proudced by IFCjs
        let targetPoints = await getPointsFromPointcloud(targetPointcloud,depth);
        // Weights are applied during the transformation stage, not the matchup stage.
        let weights = sourcePoints.map(sourcePoint=>weightEvaluation(sourceModel,sourcePoint))
        DoSingleScaleICPIterations(sourcePoints,targetPoints,weights,thresholds,result);
    }
    return result.transformation
}

function DoSingleScaleICPIterations(sourcePoints,targetPoints,weights,thresholds,result) {
    // Turn sourcePoints into a transformable matrix with the current transformation applied to it.
    let sourceMatrix = result.transformation.mmul(convertPointsArrayToMatrix(sourcePoints))

    // Finds correspondences, and the fitness and inlier_rmse of the matchup.
    UpdateRegistrationResult(sourceMatrix,targetPoints,thresholds.maxNeighborDistance,result)

    console.log("Before",result.inlier_rmse,result.fitness,result.scale)
    for(let i = 0; i < thresholds.numIterations; i++) {
        prev_fitness = result.fitness
        prev_inlier_rmse = result.inlier_rmse
        
        // Maybe perfect fit, or maybe none of the source points found neighbors, or maybe the cloud shrank to a single point.
        // In all cases, further progress is impossible.
        if(result.inlier_rmse == 0)
            break
        
        // Compute transformation
        let [tempTransform,tempScale] = ComputeTransformationPointToPoint(result.correspondences,weights)

        if(isNaN(result.scale) || result.scale === Infinity || result.scale === -Infinity)
          break
        
        // Apply transformation
        result.transformation = tempTransform.mmul(result.transformation)
        sourceMatrix = tempTransform.mmul(sourceMatrix)
        result.scale *= tempScale

        // Check convergence thresholds to determine if transformation stopped improving at this depth.
        if(i != 0 && Math.abs(prev_fitness - result.fitness) < thresholds.convergenceFitness 
            && Math.abs(prev_inlier_rmse - result.inlier_rmse) < thresholds.convergenceRMSE)
                break
        
        UpdateRegistrationResult(sourceMatrix,targetPoints,thresholds.maxNeighborDistance,result)
    }
    console.log("After",result.inlier_rmse,result.fitness,result.scale)
}

// Packages results of MatchNeighbors in a result object.
// fitness: % of the source points that found neighbors.
// inlier_rmse: average squared distance between source points that found neighbors, and their neighbors.
function UpdateRegistrationResult(sourceMatrix,targetPoints,maxNeighborDistance,result) {
    let squaredError;
    [result.correspondences,squaredError] = MatchNeighbors(sourceMatrix,targetPoints,maxNeighborDistance,result.scale)
    let numCorrespondences = result.correspondences[0].length
  
    result.fitness = numCorrespondences/sourceMatrix.columns
    result.inlier_rmse = (numCorrespondences == 0 ? 0 : Math.sqrt(squaredError/numCorrespondences))
  }
  
  // Finds neighbors for each source point in the target points. Also calculates sum of squared error, and number of successful matchings.
  function MatchNeighbors(sourceMatrix, targetPoints, maxNeighborDistance,scale) {
    let squaredError = 0
    let dims = sourceMatrix.rows - 1
    let correspondences = [[],[]]
    for(let i = 0; i < sourceMatrix.columns; i++) {
        let sourcePoint = sourceMatrix.getColumn(i).slice(0,-1)
        let closestNeighbor = {point:null,squared_distance:maxNeighborDistance ** 2}
        targetPoints.forEach(targetPoint => {
            let squared_distance = 0
            for(let dim = 0; dim < dims; dim++)
                squared_distance += ((sourcePoint[dim]-targetPoint[dim])/2)**2
            if(squared_distance < closestNeighbor.squared_distance)
                closestNeighbor = {point:targetPoint,squared_distance:squared_distance}
        })
        if(closestNeighbor.point) {
            correspondences[0].push(sourcePoint)
            correspondences[1].push(closestNeighbor.point)
            squaredError += closestNeighbor.squared_distance
        }
    }
    return [correspondences,squaredError]
  }

function convertPointsArrayToMatrix(points) {
    let numPoints = points.length;
    // See MultiScaleICP function for switching between vertex points and plane points.
    // let dims = points[0].Coordinates.length; // For vertex points
    let dims = points[0].length; // For plane points
    let matrix = new Matrix(dims,numPoints);
    for(let c = 0; c < numPoints; c++)
        // matrix.setColumn(c,points[c].Coordinates) // For vertex points
        matrix.setColumn(c,points[c]) // For plane points
    // This bottom row filled with 1s is needed for 4x4 transformation matrices to be applicable to the array.
    matrix.addRow(matrix.rows,Matrix.ones(1,numPoints));
    return matrix;
}

function ComputeTransformationPointToPoint(correspondences,weights) {
    let numPoints = correspondences[0].length
    let dims = correspondences[0][0].length
    let source_mat = new Matrix(dims,numPoints);
    let target_mat = new Matrix(dims,numPoints);

    for(let i = 0; i < numPoints; i++) {
        source_mat.setColumn(i, correspondences[0][i]);
        target_mat.setColumn(i, correspondences[1][i]);
    }

    return getSimilarityTransformation(source_mat,target_mat,weights);
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
