import {IfcAPI, IFCCARTESIANPOINT} from "web-ifc/web-ifc-api";
import { Vector3 } from "../libs/three.js/build/three.module";
import { sumPreservingRound } from "./utils";

export function getVertexPoints(schematicModel) {
  return getPointsFromMesh(schematicModel);
}

export function getRandomSurfacePoints(N,schematicModel) {
  return getMeshSurfacePoints(N,schematicModel,randomSample)
}

export function getDistributedSurfacePoints(N,schematicModel,firstSeed) {
  return getMeshSurfacePoints(N,schematicModel,distributedSample,firstSeed)
}

// Samples points from the planes in the given model.
function getMeshSurfacePoints(N,schematicModel,sampleMethod,firstSeed=0) {
  // Get triangles from mesh, and find their total area.
  let setOfTriangles = getTrianglesFromMesh(schematicModel);
  const totalArea = setOfTriangles.reduce((sum,triangle)=>sum+areaOfTriangle(...triangle),0);

  // If there's no surface area in the mesh, cannot sample points.
  if(totalArea <= 0)
    return [];

  // For each triangle, if it has x% of the surface area, give it x% of the sampled points.
  let pointCounts = sumPreservingRound(setOfTriangles.map(triangle => N * areaOfTriangle(...triangle) / totalArea));

  // Sample points according to pointCounts.
  let points = [];
  let seed = firstSeed;
  for(let i = 0; i < setOfTriangles.length; i++)
    for(let j = 0; j < pointCounts[i]; j++) {
      points.push(samplePointFromTriangle(...(setOfTriangles[i]),sampleMethod,seed++));
      seed++;
    }

  return points;
}

// Obtains the triangles of the model created by IFCLoader or OBJLoader
function getTrianglesFromMesh(schematicModel) {
  let points = getPointsFromMesh(schematicModel);
  let triangles = [];
  let index = schematicModel.geometry.index?.array;
  if(index)
    for(let i = 0; i <= index.length - 3; i += 3)
      triangles.push([points[index[i]],points[index[i+1]],points[index[i+2]]])
  else
    for(let i = 0; i <= points.length - 3; i += 3)
      triangles.push([points[i],points[i+1],points[i+2]])
  return triangles
}

// Obtains the vertices of the model created by IFCLoader or OBJLoader
function getPointsFromMesh(schematicModel) {
  let array = schematicModel.geometry.attributes.position.array;
  let points = [];
  for(let i = 0; i+2 < array.length; i += 3) {
    let point = new Vector3(array[i], array[i+1], array[i+2]);
    point.applyMatrix4(schematicModel.matrix);
    points.push([point.x, point.y, point.z]);
  }
  return points;
}


// Obtains a random point from the unit triangle
function randomSample() {
  let x = Math.random();
  let y = Math.random();
  if(x+y>1)
    [x,y] = [1-x,1-y];
  return [x,y]
}

// Seed should be at least 0. Low seeds can be problematic.
export function distributedSample(seed) {
  // https://en.wikipedia.org/wiki/Plastic_number
  const plastic = Math.cbrt((9+Math.sqrt(69))/18) + Math.cbrt((9-Math.sqrt(69))/18);

  // https://web.archive.org/web/20190606192126/https://extremelearning.com.au/evenly-distributing-points-in-a-triangle/
  let [x,y] = [(seed/plastic)%1,(seed/plastic**2)%1];
  if(x+y>1)
    [x,y] = [1-x,1-y];

  return [x,y]
}

function samplePointFromTriangle(pointA,pointB,pointC,unitTriangleSampleMethod,seed) {
  // unitTriangleSampleMethod should return a point in the triangle [(0,0), (1,0), (0,1)]
  let [x,y] = unitTriangleSampleMethod(seed)
  
  // Relabel given triangle so newA is opposite the longest side
  let AB=0,AC=0,BC=0;
  for(let dim = 0; dim < pointA.length; dim++) {
    AB += (pointB[dim]-pointA[dim]) ** 2;
    AC += (pointC[dim]-pointA[dim]) ** 2;
    BC += (pointC[dim]-pointB[dim]) ** 2;
  }
  let newA,newB,newC;
  if(AB > AC && AB > BC)
    newA=pointC,newB=pointB,newC=pointA;
  else if(AC > BC)
    newA=pointB,newB=pointA,newC=pointC;
  else
    newA=pointA,newB=pointB,newC=pointC;
  
  // Transform the points from unit triangle to the given triangle.
  // Stretches the triangle basically like tilting it in perspective, so uniformity should be preserved.
  let point = [];
  for(let dim = 0; dim < pointA.length; dim++)
    // A + xAB + yAC
    point.push(newA[dim] + x*(newB[dim]-newA[dim]) + y*(newC[dim]-newA[dim]));
  
  // IFCJS is rotated 90 degrees relative to potree. Correction:
  let temp = point[2];
  point[2] = point[1];
  point[1] = -1 * temp;


  return new Vector3(...point);
}

// Get area of triangle
// https://en.wikipedia.org/wiki/Area_of_a_triangle#Using_vectors
function areaOfTriangle(pointA,pointB,pointC) {
  let ABdotAB=0,ABdotAC=0,ACdotAC=0
  for(let dim = 0; dim < pointA.length; dim++) {
    ABdotAB+=(pointB[dim]-pointA[dim])**2
    ABdotAC+=(pointB[dim]-pointA[dim])*(pointC[dim]-pointA[dim])
    ACdotAC+=(pointC[dim]-pointA[dim])**2
  }
  return Math.sqrt(ABdotAB*ACdotAC - ABdotAC**2)/2
}


// getPointsFromMesh seems like a better alternative to this function.
// Takes IFCCARTESIANPOINTs from IFC file. 
// Assuming all vertices use by IFCLoader are cartesianpoints; uncertain.
// Should not be run on a filePath which does not end in .ifc
/*
export async function getVertexPoints(ifcFilePath) {
  // Verify IFC filetype.
  let filePathExtension = filePath.split('.').pop();
  if(!filePathExtension || "ifc" !== filePathExtension.toLowerCase())
      throw new Error("Unsupported: attempting to read IFCCARTESIANPOINT vertices from a non-ifc file.");

  let modelID = 0;

  console.log(ifcFilePath)

  function getIfcFile(url) {
      return new Promise((resolve,reject) => {
          var oReq = new XMLHttpRequest();
          oReq.responseType = "arraybuffer";
          oReq.addEventListener("load", () => {
              resolve(new Uint8Array(oReq.response));
          });
          oReq.open("GET", url);
          oReq.send();
      });
  }

  const ifcapi = new IfcAPI();
  let ifcPoints = [];
  await ifcapi.Init().then(()=>getIfcFile(ifcFilePath)).then((ifcData) => {
      modelID = ifcapi.OpenModel(ifcData);
      let lines = ifcapi.GetLineIDsWithType(modelID,IFCCARTESIANPOINT);
      
      for(let i = 0; i < lines.size(); i++)
          ifcPoints.push(ifcapi.GetLine(modelID,lines.get(i)));
  });

  return ifcPoints;
}
*/