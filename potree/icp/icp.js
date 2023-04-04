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

import {Matrix} from "ml-matrix"
import { getAllPoints } from "./potreeAccess";
import { getSimilarityTransformation } from "./umeyama";

// Tweakable
// How deep into the potree the ICP should go. 0 means it only checks the root's points.
const MAX_DEPTH = 5

// Tweakable
// At each depth:
    // max_iterations: how many iterations can be done at this depth at most.
    // convergence_fitness and convergence_rmse: if change in fitness and rmse both get under their convergence thresholds,
    // then this depth has basically stopped changing, so move to the next depth. 
    // max_neighbor_distance: how far to search for nearby neighbors to points.
function getThresholds(depth) {
    let max_iterations = 1**(MAX_DEPTH-depth+1);
    
    let convergence_fitness = 0;
    let convergence_rmse = 0;
    
    let max_neighbor_distance = Infinity;

    return [max_iterations,convergence_fitness,convergence_rmse,max_neighbor_distance];
}



// The main function
export async function MultiScaleICP(sourceArray, targetPointcloud, initialEstimate) {
    let result = {transformation:initialEstimate, correspondences:[], fitness:0, inlier_rmse:0}

    // Apply initial estimate to the source points
    let sourceMatrix = initialEstimate.mmul(convertIFCArrayToMatrix(sourceArray))

    for(i = 0; i <= MAX_DEPTH; i++)
        [result,sourceMatrix] = await DoSingleScaleICPIterations(sourceMatrix,targetPointcloud,i,result)

    await UpdateRegistrationResult(sourceMatrix,targetPointcloud,getThresholds(MAX_DEPTH)[3],MAX_DEPTH,result)
    return result.transformation
}

/*
function validateInput(source, target, initialEstimate){
    // Neither target nor source must be empty
    if(source.size() <= 0 || target.size() <= 0)
        return false;

    // initialEstimate must be a transformation matrix for source.
    let dims = source[0].size();
    if(initialEstimate.rows != dims + 1 || initialEstimate.columns != dims + 1)
        return false;
    for(let i = 0; i < dims; i++)
        if(initialEstimate.getRow(dims)[i] != 0)
            return false;
    if(initialEstimate.getRow(dims)[dims] != 1)
        return false;
    
    return true;
}
*/

async function DoSingleScaleICPIterations(sourceMatrix,targetPointcloud,depth,result) {
    let [max_iterations,convergence_fitness,convergence_rmse,max_neighbor_distance] = getThresholds(depth)
    
    for(let i = 0; i < max_iterations; i++) {
        prev_fitness = result.fitness
        prev_inlier_rmse = result.inlier_rmse
        
        // Finds correspondences, also computes fitness and inlier_rmse of the matchup.
        await UpdateRegistrationResult(sourceMatrix,targetPointcloud,max_neighbor_distance,depth,result)
        // Maybe perfect fit, or maybe none of the source points found neighbors, or maybe the cloud shrank to a single point.
        // In all cases, further progress is impossible.
        if(result.inlier_rmse == 0)
            return [result,sourceMatrix]
        
        // Compute and apply transformation
        let update = ComputeTransformationPointToPoint(result.correspondences)
        result.transformation = update.mmul(result.transformation)
        sourceMatrix = update.mmul(sourceMatrix)

        // Check thresholds to determine if transformation stopped improving at this depth.
        if(i != 0 && Math.abs(prev_fitness - result.fitness) < convergence_fitness 
            && Math.abs(prev_inlier_rmse - result.inlier_rmse) < convergence_rmse)
                break
    }
    return [result,sourceMatrix]
}

// Packages results of MatchNeighbors in a result object.
// fitness: % of the source points that found neighbors.
// inlier_rmse: average squared distance between source points that found neighbors, and their neighbors.
async function UpdateRegistrationResult(sourceMatrix,targetPointcloud,max_neighbor_distance,depth,result) {
    let squaredError
    [result.correspondences,squaredError] = await MatchNeighbors(sourceMatrix,targetPointcloud,max_neighbor_distance,depth)
    let numCorrespondences = result.correspondences[0].length
  
    result.fitness = numCorrespondences/sourceMatrix.columns
    result.inlier_rmse = (numCorrespondences == 0 ? 0 : Math.sqrt(squaredError/numCorrespondences))
  }
  
  // Finds neighbors for each source point in the target points. Also calculates sum of squared error, and number of successful matchings.
  async function MatchNeighbors(sourceMatrix, targetPointcloud, max_neighbor_distance, depth) {
    // Spacing is the minimum distance between points in the node.
    let targetPoints = await getAllPoints(targetPointcloud,depth)
    let squaredError = 0
    let correspondences = [[],[]]
    for(let c = 0; c < sourceMatrix.columns; c++) {
        let sourcePoint = sourceMatrix.getColumn(c);
        let closestNeighbor = {point:null,squared_distance:Infinity}
        targetPoints.forEach(targetPoint=>{
            let squared_distance = 0
            // length - 1 to ignore the row of 1s
            for(let i = 0; i < sourcePoint.length - 1; i++)
                squared_distance += (sourcePoint[i]-targetPoint[i])**2
            if(squared_distance < closestNeighbor.squared_distance)
                closestNeighbor = {point:targetPoint,squared_distance:squared_distance}
        })
        if(closestNeighbor.squared_distance <= max_neighbor_distance**2) {
            correspondences[0].push(sourcePoint)
            correspondences[1].push(closestNeighbor.point)
            squaredError += closestNeighbor.squared_distance
        }
    }
    return [correspondences,squaredError]
  }

function convertIFCArrayToMatrix(points) {
    let numPoints = points.length;
    let dims = points[0].Coordinates.length;
    let matrix = new Matrix(dims,numPoints);
    for(let c = 0; c < numPoints; c++)
        matrix.setColumn(c,points[c].Coordinates.map(x=>x.value))
    // This bottom row filled with 1s is needed for 4x4 transformation matrices to be applicable to the array.
    matrix.addRow(matrix.rows,Matrix.ones(1,numPoints));
    return matrix;
}

// Obtain the best transformation matrix A so that A*correspondences[][0] is very close to correspondences[][1]
function ComputeTransformationPointToPoint(correspondences) {
    let numPoints = correspondences[0].length
    let dims = correspondences[1][0].length
    let source_mat = new Matrix(dims,numPoints);
    let target_mat = new Matrix(dims,numPoints);

    for(let i = 0; i < numPoints; i++) {
        source_mat.setColumn(i, correspondences[0][i].slice(0,dims));
        target_mat.setColumn(i, correspondences[1][i]);
    }

    return getSimilarityTransformation(source_mat,target_mat);
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
