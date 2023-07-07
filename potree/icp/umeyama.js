/**
https://github.com/batzner/similarity-transformation-js

MIT License

Copyright (c) 2020 Kilian Batzner

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

/**
 * @fileoverview Implementation of the algorithm of Shinji Umeyama for matching
 * two tuples of n-dimensional points through rotation, translation and
 * scaling. This implementation also adds the possibility to allow reflecting
 * one of the tuples to achieve a better matching.
 *
 * The central function is getSimilarityTransformation(...).
 *
 * This file depends on the ml-matrix library for computing matrix
 * manipulations, see https://github.com/mljs/matrix. Version used: 6.4.1
 *
 * umeyama_1991 refers to http://web.stanford.edu/class/cs273/refs/umeyama.pdf:
 * @article{umeyama_1991,
 *   title={Least-squares estimation of transformation parameters between two point patterns},
 *   author={Umeyama, Shinji},
 *   journal={IEEE Transactions on Pattern Analysis \& Machine Intelligence},
 *   number={4},
 *   pages={376--380},
 *   year={1991},
 *   publisher={IEEE}
 * }
 *
 * Variable names and the corresponding term in the paper's notation:
 * - fromPoints refers to {x_i} with i = 1, 2, ..., n
 * - toPoints refers to {y_i} with i = 1, 2, ..., n
 * - numPoints refers to n
 * - dimensions refers to m
 * - rotation refers to R
 * - scale refers to c
 * - translation refers to t
 * - fromMean and toMean refer to mu_x and mu_y respectively
 * - fromVariance and toVariance refer to sigma_x and sigma_y respectively
 * - mirrorIdentity refers to S
 * - svd refers to the SVD given by U, D and V
 */

import {determinant,Matrix as Mlmatrix,SVD} from "ml-matrix"

/**
 * Compute transformation to transform a tuple of source points to match a tuple of target points
 * following equation 40, 41 and 42 of umeyama_1991.
 *
 * This function expects two Mlmatrix instances of the same shape
 * (m, n), where n is the number of points and m is the number of dimensions.
 * This is the shape used by umeyama_1991. m and n can take any positive value.
 * 
 * Added weight function to treat certain columns as being present multiple times, or fractional numbers.
 * For floating point, works best if weights are near 1.
 * Weights array should be length n.
 *
 * The returned matrix would transform fromPoints to toPoints.
 *
 * @param {!Mlmatrix} fromPoints - the source points {x_1, ..., x_n}.
 * @param {!Mlmatrix} toPoints - the target points {y_1, ..., y_n}.
 * @param {boolean} allowReflection - If true, the source points may be
 *   reflected to achieve a better mean squared error.
 * @returns {Mlmatrix}
 */
export function getSimilarityTransformation(fromPoints,
                                     toPoints,
                                     weights,
                                     allowReflection = false) {
    const dimensions = fromPoints.rows;

    // 1. Compute the rotation.
    const covarianceMatrix = getSimilarityTransformationCovariance(
        fromPoints,
        toPoints,
        weights);

    const {
        svd,
        mirrorIdentityForSolution
    } = getSimilarityTransformationSvdWithMirrorIdentities(
        covarianceMatrix,
        allowReflection);

    const rotation = svd.U
        .mmul(Mlmatrix.diag(mirrorIdentityForSolution))
        .mmul(svd.V.transpose());

    // 2. Compute the scale.
    // The variance will first be a 1-D array and then reduced to a scalar.
    const summator = (sum, elem) => {
        return sum + elem;
    };

    const toVariance = weightedVarianceByRow(toPoints,weights); // fromPoints was incorrect and is now replaced with toPoints

    let trace = 0;
    for (let dimension = 0; dimension < dimensions; dimension++) {
        const mirrorEntry = mirrorIdentityForSolution[dimension];
        trace += svd.diagonal[dimension] * mirrorEntry;
    }
    const scale = toVariance / trace; // Bugfix

    // 3. Compute the translation.
    const fromMean = Mlmatrix.columnVector(weightedMeanByRow(fromPoints,weights))
    const toMean = Mlmatrix.columnVector(weightedMeanByRow(toPoints,weights))
    const translation = Mlmatrix.sub(
        toMean,
        Mlmatrix.mul(rotation.mmul(fromMean), scale));

    /*
    // 4. Transform the points.
    const transformedPoints = Mlmatrix.add(
        Mlmatrix.mul(rotation.mmul(fromPoints), scale),
        translation.repeat({columns: numPoints}));

    return transformedPoints;
    */

    const transformationMatrix = rotation.mul(scale)
    transformationMatrix.addColumn(dimensions,translation)
    transformationMatrix.addRow(dimensions,Mlmatrix.zeros(1,dimensions + 1))
    transformationMatrix.set(dimensions,dimensions,1)

    // Scale is useful to return also
    return [transformationMatrix,scale];
}

/**
 * Compute the mean squared error of a given solution, following equation 1
 * in umeyama_1991.
 *
 * This function expects two Mlmatrix instances of the same shape
 * (m, n), where n is the number of points and m is the number of dimensions.
 * This is the shape used by umeyama_1991.
 *
 * @param {!Mlmatrix} transformedPoints - the solution, for example
 *   returned by getSimilarityTransformation(...).
 * @param {!Mlmatrix} toPoints - the target points {y_1, ..., y_n}.
 * @returns {number}
 */
function getSimilarityTransformationError(transformedPoints, toPoints) {
    const numPoints = transformedPoints.columns;
    const difference = Mlmatrix.sub(toPoints, transformedPoints);
    return Math.pow(difference.norm('frobenius'), 2) / numPoints;
}

/**
 * Compute the minimum possible mean squared error for a given problem,
 * following equation 33 in umeyama_1991.
 *
 * This function expects two Mlmatrix instances of the same shape
 * (m, n), where n is the number of points and m is the number of dimensions.
 * This is the shape used by umeyama_1991. m and n can take any positive value.
 *
 * @param {!Mlmatrix} fromPoints - the source points {x_1, ..., x_n}.
 * @param {!Mlmatrix} toPoints - the target points {y_1, ..., y_n}.
 * @param {boolean} allowReflection - If true, the source points may be
 *   reflected to achieve a better mean squared error.
 * @returns {number}
 */
function getSimilarityTransformationErrorBound(fromPoints,
                                               toPoints,
                                               allowReflection = false) {
    const dimensions = fromPoints.rows;

    // The variances will first be 1-D arrays and then reduced to a scalar.
    const summator = (sum, elem) => {
        return sum + elem;
    };
    const fromVariance = fromPoints
        .variance('row', {unbiased: false})
        .reduce(summator);
    const toVariance = toPoints
        .variance('row', {unbiased: false})
        .reduce(summator);
    const covarianceMatrix = getSimilarityTransformationCovariance(
        fromPoints,
        toPoints);

    const {
        svd,
        mirrorIdentityForErrorBound
    } = getSimilarityTransformationSvdWithMirrorIdentities(
        covarianceMatrix,
        allowReflection);

    let trace = 0;
    for (let dimension = 0; dimension < dimensions; dimension++) {
        const mirrorEntry = mirrorIdentityForErrorBound[dimension];
        trace += svd.diagonal[dimension] * mirrorEntry;
    }
    return toVariance - Math.pow(trace, 2) / fromVariance;
}

/**
 * Computes the covariance matrix of the source points and the target points
 * following equation 38 in umeyama_1991.
 *
 * This function expects two Mlmatrix instances of the same shape
 * (m, n), where n is the number of points and m is the number of dimensions.
 * This is the shape used by umeyama_1991. m and n can take any positive value.
 * 
 * Note: I have added weighting to this function. It acts as if there are weights[i] copies of each point.
 *
 * @param {!Mlmatrix} fromPoints - the source points {x_1, ..., x_n}.
 * @param {!Mlmatrix} toPoints - the target points {y_1, ..., y_n}.
 * @returns {Mlmatrix}
 */
function getSimilarityTransformationCovariance(fromPoints, toPoints, weights) {
    const dimensions = fromPoints.rows;
    const numPoints = fromPoints.columns;

    let totalWeight = 0;
    for(let j = 0; j < weights.length; ++j)
        totalWeight += weights[j];

    const fromMean = Mlmatrix.columnVector(weightedMeanByRow(fromPoints,weights));
    const toMean = Mlmatrix.columnVector(weightedMeanByRow(toPoints,weights));

    const covariance = Mlmatrix.zeros(dimensions, dimensions);

    for (let pointIndex = 0; pointIndex < numPoints; pointIndex++) {
        const fromPoint = fromPoints.getColumnVector(pointIndex);
        const toPoint = toPoints.getColumnVector(pointIndex);
        const outer = Mlmatrix.sub(toPoint, toMean)
            .mmul(Mlmatrix.sub(fromPoint, fromMean).transpose());

        covariance.addM(Mlmatrix.div(outer.mulS(weights[pointIndex]), totalWeight));
    }

    return covariance;
}

/**
 * Computes the SVD of the covariance matrix and returns the mirror identities
 * (called S in umeyama_1991), following equation 39 and 43 in umeyama_1991.
 *
 * See getSimilarityTransformationCovariance(...) for more details on how to
 * compute the covariance matrix.
 *
 * @param {!Mlmatrix} covarianceMatrix - the matrix returned by
 *   getSimilarityTransformationCovariance(...)
 * @param {boolean} allowReflection - If true, the source points may be
 *   reflected to achieve a better mean squared error.
 * @returns {{
 *   svd: SVD,
 *   mirrorIdentityForErrorBound: number[],
 *   mirrorIdentityForSolution: number[]
 * }}
 */
function getSimilarityTransformationSvdWithMirrorIdentities(covarianceMatrix,
                                                            allowReflection) {
    // Compute the SVD.
    const dimensions = covarianceMatrix.rows;
    const svd = new SVD(covarianceMatrix);

    // Compute the mirror identities based on the equations in umeyama_1991.
    let mirrorIdentityForErrorBound = Array(svd.diagonal.length).fill(1);
    let mirrorIdentityForSolution = Array(svd.diagonal.length).fill(1);
    if (!allowReflection) {
        // Compute equation 39 in umeyama_1991.
        if (determinant(covarianceMatrix) < 0) {
            const lastIndex = mirrorIdentityForErrorBound.length - 1;
            mirrorIdentityForErrorBound[lastIndex] = -1;
        }

        // Check the rank condition mentioned directly after equation 43.
        mirrorIdentityForSolution = mirrorIdentityForErrorBound;
        if (svd.rank === dimensions - 1) {
            // Compute equation 43 in umeyama_1991.
            mirrorIdentityForSolution = Array(svd.diagonal.length).fill(1);
            if (determinant(svd.U) * determinant(svd.V) < 0) {
                const lastIndex = mirrorIdentityForSolution.length - 1;
                mirrorIdentityForSolution[lastIndex] = -1;
            }
        }
    }

    return {
        svd: svd,
        mirrorIdentityForErrorBound: mirrorIdentityForErrorBound,
        mirrorIdentityForSolution: mirrorIdentityForSolution
    }
}






function weightedVarianceByRow(matrix, weights) {
    let totalWeight = 0;
    for(let j = 0; j < matrix.columns; ++j)
        totalWeight += weights[j];

    const mean = weightedMeanByRow(matrix, weights);

    let variance = 0;
    for (let i = 0; i < matrix.rows; i++) {
        let sum1 = 0;
        let sum2 = 0;
        for (let j = 0; j < matrix.columns; j++) {
            const x = matrix.get(i, j) - mean[i];
            sum1 += x * weights[j];
            sum2 += x * x * weights[j];
        }
        variance += (sum2 - (sum1 * sum1) / totalWeight) / totalWeight;
    }
    return variance;
}


function weightedMeanByRow(matrix,weights) {
    let totalWeight = 0;
    for(let j = 0; j < matrix.columns; ++j)
        totalWeight += weights[j];

    const mean = [];
    for (let i = 0; i < matrix.rows; ++i) {
        let val = 0;
        for (let j = 0; j < matrix.columns; ++j)
            val += matrix.get(i, j) * weights[j];
        val /= totalWeight;
        mean.push(val)
    }
    return mean;
}