import { LinearMipMapLinearFilter, Matrix4 } from "../libs/three.js/build/three.module";
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
// If one array has more rows, use the higher amount of rows.
export function mlmatrixConcat(mlmatrixA, mlmatrixB) {
    let rowsA = mlmatrixA.rows;
    let rowsB = mlmatrixB.rows;
    let newData = new Array(Math.max(rowsA,rowsB));
    for(let r = 0; r < newData.length; r++) {
        newData[r] = [];
        if(r < rowsA)
            newData[r].push(...(mlmatrixA.data[r]));
        if(r < rowsB)
            newData[r].push(...(mlmatrixB.data[r]));
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
// Returns the index of the median of the list. 
// Errors if list is empty.
export function medianFind(list,comparison) {
    return sortedIndexFind(list, Math.floor(list.length/2-0.5), comparison);
}

// https://rcoh.me/posts/linear-time-median-finding/
// Returns the index of the item which, if the list were sorted, would be located at the given index.
// Errors if list is empty.
function sortedIndexFind(list,index,comparison) {
    if(list.length == 1)
        return index;
    let pivotIndex = Math.floor(Math.random() * list.length);
    let left = [];
    let right = [];
    list.forEach(item => {
        let compValue = comparison(item,list[pivotIndex])
        if(compValue < 0)
            left.push(item)
        if(compValue > 0)
            right.push(item)
    })
    if(index < left.length)
        return sortedIndexFind(left,index,comparison);
    let rightStart = list.length - right.length;
    if(index >= rightStart)
        return rightStart + sortedIndexFind(right,index-rightStart,comparison);
    return pivotIndex;
}
