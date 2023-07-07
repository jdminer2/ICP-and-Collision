import { Matrix4 } from "../libs/three.js/build/three.module";
import { Matrix as Mlmatrix } from "ml-matrix";

export function mlmatrixToMatrix4(mlmatrix) {
    return (new Matrix4()).set(...(mlmatrix.to1DArray()));
}

export function matrix4ToMlmatrix(threematrix) {
    return Mlmatrix.from1DArray(4,4,threematrix.toArray()).transpose();
}

export async function waitForDef(getter) {
    let val = getter();
    while(val === undefined || val === null) {
        await new Promise(e=>setTimeout(e,0));
        val = getter();
    }
    return val
}