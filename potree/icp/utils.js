import { Matrix4 } from "../libs/three.js/build/three.module";
import { Matrix as Mlmatrix } from "ml-matrix";

export function mlmatrixToMatrix4(mlmatrix) {
    return (new Matrix4()).set(...(mlmatrix.to1DArray()));
}

export function matrix4ToMlmatrix(threematrix) {
    return Mlmatrix.from1DArray(4,4,threematrix.toArray()).transpose();
}

// Each item in the array2D turns into a row of the mlmatrix. 
// It is assumed that each column of the array2D/mlmatrix represents a point.
// An extra row of 1s is added for transformation.
export function array2DToMlmatrix(array2D) {
    let matrix = new Mlmatrix(array2D);
    // This bottom row filled with 1s is needed for 4x4 transformation matrices to be applicable to the array.
    matrix.addRow(matrix.rows,Mlmatrix.ones(1,matrix.columns));
    return matrix;
}

// Creates a transposed version of array2D, assuming that all row lengths are equal to the first row length.
// Does not modify or return something pointing to the original array.
export function transposeArray2D(array2D) {
    let rows = array2D.length;
    if(rows == 0)
        return array2D;
    let cols = array2D[0].length;

    let newArray2D = new Array(cols);
    for(let c = 0; c < cols; c++) {
        newArray2D[c] = new Array(rows);
        for(let r = 0; r < rows; r++)
            newArray2D[c][r] = array2D[r][c];
    }
    return newArray2D;
}

// Copies instead of modifying. 
// A's columns are on the left, B's columns are on the right.
// If one array has more rows, fill all-zero rows in the smaller one to match.
export function mlmatrixConcat(mlmatrixA, mlmatrixB) {
    let newData = new Array(Math.max(mlmatrixA.rows,mlmatrixB.rows));
    for(let r = 0; r < newData.length; r++) {
        newData[r] = new ((r < mlmatrixA.rows) ? mlmatrixA : mlmatrixB).data[r].constructor(mlmatrixA.columns + mlmatrixB.columns);
        if(r < mlmatrixA.rows)
            newData[r].set(mlmatrixA.data[r]);
        if(r < mlmatrixB.rows)
            newData[r].set(mlmatrixB.data[r], mlmatrixA.columns);
    }
    return new Mlmatrix(newData);
}

export async function waitForDef(getter) {
    let val = getter();
    while(val === undefined || val === null) {
        await new Promise(e=>setTimeout(e,0));
        val = getter();
    }
    return val
}

// Modifies the original list to sort it; returns nothing.
// comparison(a,b) < 0 -> a before b. comparison(a,b) > 0 -> a after b. comparison(a,b) == 0 -> keep original relative order.
export async function mergeSort(list, comparison) {
    mergeSortHelper(list, comparison, 0, list.length);
}

async function mergeSortHelper(list, comparison, start, afterEnd) {
    // Already sorted
    if(afterEnd - start <= 1)
        return;
    let middle = Math.floor((start + afterEnd)/2);
    // Recursively sort both halves of the list.
    mergeSortHelper(list,start,middle);
    mergeSortHelper(list,middle,afterEnd);
    // Merge the halves
    let lefti = start;
    let righti = middle;
    let newList = new Array(afterEnd - start);
    for(let i = 0; i < newList.length; i++) {
        if(lefti >= middle)
            newList[i] = list[righti++];
        else if(righti >= afterEnd)
            newList[i] = list[lefti++];
        else if(comparison(list[lefti],list[righti]) <= 0)
            newList[i] = list[lefti++];
        else
            newList[i] = list[righti++];
    }
    // Insert back into the list.
    for(let i = 0; i < newList.length; i++)
        list[i + start] = newList[i]
}



// https://rcoh.me/posts/linear-time-median-finding/
// Returns the median of the array. 
// Errors if list is empty.
export function medianFind(array,comparison) {
    return sortedIndexFind(array, Math.floor(array.length/2-0.5), comparison);
}

// https://rcoh.me/posts/linear-time-median-finding/
// Returns the item which, if the array were sorted, would be located at the given index.
// Errors if array is empty.
function sortedIndexFind(array,index,comparison) {
    let pivotIndex = Math.floor(Math.random() * array.length);
    let left = [];
    let right = [];
    array.forEach(item => {
        let compValue = comparison(item,array[pivotIndex])
        if(compValue < 0)
            left.push(item)
        if(compValue > 0)
            right.push(item)
    })
    if(index < left.length)
        return sortedIndexFind(left,index,comparison);
    let rightStart = array.length - right.length;
    if(index >= rightStart)
        return sortedIndexFind(right,index-rightStart,comparison);
    return array[pivotIndex];
}

// Returns an array of n integers in the range 0(inclusive) to len(exclusive) with no repeats.
// len<0 is treated as len=0. n<0 is treated as n=0. n>len is treated as n=len.
export function arrayRandomSubset(n,len) {
    // Sampling 0 items or from nothing returns nothing.
    if(n <= 0 || len <= 0)
        return [];

    // Make array of 0(inclusive) to len(exclusive).
    let array = Array(len);
    let i = 0;
    while(i < len)
        array[i] = i++;

    // Sampling everything returns everything.
    if(n >= len)
        return array;

    // Partial Fisher Yates shuffle with the last n items in the list.
    for(i = 1; i <= n; i++) {
        // idx is the ith-last index in the array.
        let idx = len-i;
        // Pick 0 <= j <= idx randomly.
        let j = Math.floor(Math.random()*(idx+1));
        // Swap j and idx.
        let temp = array[idx];
        array[idx] = array[j];
        array[j] = temp;
    }
    // Return the last n items.
    return array.slice(len-n);
}

// Round an array of values up or down, such that their sum changes as little as possible,
// and a number like 1.6 does not round down if a number like 2.4 rounds up.
// The decimal parts of numbers that round up must be >= than those of those that round down.
export function sumPreservingRound(array) {
    // Find how many numbers need to be ceiled, by comparing the floor-all sum to the target sum.
    let sum = 0;
    let floorSum = 0;
    for(let i = 0; i < array.length; i++) {
        sum += array[i];
        floorSum += Math.floor(array[i]);
    }
    sum = Math.round(sum);
    let ceils = sum - floorSum;

    // Find the cutoff point for floor vs ceil.
    let lowestCeil;
    if(ceils == 0)
        lowestCeil = 1.0;
    else
        lowestCeil = sortedIndexFind(array, array.length - ceils, (a,b) => a%1 - b%1) % 1;

    // Ceil numbers above the cutoff, floor numbers on or below the cutoff.
    let roundedArray = new Array(array.length);
    for(let i = 0; i < array.length; i++) {
        if(array[i] % 1 > lowestCeil) {
            roundedArray[i] = Math.ceil(array[i]);
            ceils--;
        }
        else
            roundedArray[i] = Math.floor(array[i]);
    }

    // For numbers on the cutoff, change some to ceil. as many as necessary to obtain the desired sum.
    for(let i = 0; i < array.length && ceils > 0; i++) {
        if(array[i] % 1 == lowestCeil) {
            roundedArray[i] = Math.ceil(array[i]);
            ceils--;
        }
    }

    return roundedArray;
}

export function updateSchematicTextboxes() {
    // viewer.js has code to translate from textbox properties to object properties.
    // This does the inverse, to translate object properties to textbox properties for inputting into setSchemParam.
    
    // Correction for flipped dimensions
    let schemBoundingBox = viewer.schematic.geometry.boundingBox.clone();
    let temp = [schemBoundingBox.min.z,schemBoundingBox.max.z];
    schemBoundingBox.min.z = -1 * schemBoundingBox.max.y;
    schemBoundingBox.max.z = -1 * schemBoundingBox.min.y;
    schemBoundingBox.min.y = temp[0];
    schemBoundingBox.max.y = temp[1];

    let textBoxScale = viewer.schematic.scale.clone().divide(viewer.schemVolume.objectScaleFactor).x;

    let objectRotation = viewer.schematic.rotation.clone();
    objectRotation.order = "XZY";
    let adjustedPositionOffset = viewer.schemVolume.positionOffset.clone().applyEuler(objectRotation).multiply(viewer.schematic.scale);

    let textboxPosition = viewer.schematic.position.clone().add(adjustedPositionOffset);
    // Update schematic textboxes.
    ["x","y","z"].forEach(dimension => {
        viewer.setSchemParam("position",dimension,textboxPosition[dimension]);
        viewer.setSchemParam("rotation",dimension,viewer.schematic.rotation[dimension]);
    });
    viewer.setSchemParam("scale","all",textBoxScale);

}

export function updatePointcloudTextboxes() {
    // viewer.js has code to translate from textbox properties to object properties.
    // This does the inverse, to translate object properties to textbox properties for inputting into setPclParam.
    let textBoxScale = viewer.pointcloud.scale.clone().divide(viewer.pclVolume.objectScaleFactor).x;
    let adjustedPositionOffset = viewer.pclVolume.positionOffset.clone().applyEuler(viewer.pointcloud.rotation).multiply(viewer.pointcloud.scale);
    let textboxPosition = viewer.pointcloud.position.clone().add(adjustedPositionOffset);
    // Update pointcloud textboxes.
    ["x","y","z"].forEach(dimension => {
        viewer.setPclParam("position",dimension,textboxPosition[dimension]);
        viewer.setPclParam("rotation",dimension,viewer.pointcloud.rotation[dimension]);
    });
    viewer.setPclParam("scale","all",textBoxScale);
}

export function initializePclCropTextboxes() {
    // Set values to equal the schemVolume tool.
    for(parameter of ["position","rotation","scale"])
        for(dimension of ["x","y","z"])
            viewer.setPclCropParam(parameter,dimension,viewer.schemVolume[parameter][dimension]);
}