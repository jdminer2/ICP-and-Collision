import {IfcAPI, IFCCARTESIANPOINT} from "web-ifc/web-ifc-api";

// Takes IFCCARTESIANPOINTs from IFC file. May be inaccurate for some IFC files.
export async function getVertexPoints(ifcFilePath) {
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

export function getRandomPlanePoints(ifcModel,pointCount) {
  return getPlanePoints(ifcModel,pointCount,randomSample)
}

export function getDistributedPlanePoints(ifcModel,pointCount) {
  return getPlanePoints(ifcModel,pointCount,distributedSample)
}

// Samples points from the planes in the given model.
function getPlanePoints(ifcModel,pointCount,sampleMethod) {
  let setOfTriangles = getSetOfTriangles(ifcModel);
  const totalArea = setOfTriangles.reduce((sum,triangle)=>sum+area(...triangle),0);

  if(totalArea <= 0)
    return [];

  let exploredArea = 0;
  let points = [];
  for(let i = 0; i < setOfTriangles.length; i++) {
    const triangle = setOfTriangles[i];
    exploredArea += area(...triangle);

    // Percent of pointCount filled should reach the percent of surface area explored so far.
    pointTarget = pointCount * Math.min(exploredArea / totalArea, 1)
    while(points.length < pointTarget)
      points.push(samplePointFromTriangle(...triangle,points.length,sampleMethod));
  }

  // Make sure the final triangle reaches pointCount (in case pointTarget didn't reach it due to rounding errors)
  while(points.length < pointCount)
    points.push(samplePointFromTriangle(...(setOfTriangles.slice(-1)),points.length,sampleMethod));

  return points;
}

export function randomSample() {
  let x = Math.random()
  let y = Math.random() * x
  return [x,y]
}

// Seed should be at least 0.
export function distributedSample(seed) {
  // https://en.wikipedia.org/wiki/Plastic_number
  const plastic = Math.cbrt((9+Math.sqrt(69))/18) + Math.cbrt((9-Math.sqrt(69))/18);

  // https://web.archive.org/web/20190606192126/https://extremelearning.com.au/evenly-distributing-points-in-a-triangle/
  let [x,y] = [((seed+20)/plastic)%1,((seed+20)/plastic**2)%1];

  if(x+y>1)
    [x,y] = [1-x,1-y];

  return [x,y]
}


function getSetOfTriangles(ifcModel) {
  let array = ifcModel.geometry.attributes.position.array;
  let dims = ifcModel.geometry.attributes.position.itemSize;
  let points = []
  for(let i = 0; i <= array.length - dims; i += dims)
    points.push(array.slice(i,i+dims));
  let index = ifcModel.geometry.index.array;
  let triangles = [];
  for(let i = 0; i <= index.length - 3; i += 3)
    triangles.push([points[index[i]],points[index[i+1]],points[index[i+2]]])
  return triangles
}

function samplePointFromTriangle(pointA,pointB,pointC,seed,unitTriangleSampleMethod) {
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
  
  // Transform the point from unit triangle to the given triangle
  let point = [];
  for(let dim = 0; dim < pointA.length; dim++)
    // A + xAB + yAC
    point.push(newA[dim] + x*(newB[dim]-newA[dim]) + y*(newC[dim]-newA[dim]));
  
  // IFCJS is rotated 90 degrees relative to potree. Correction:
  let temp = point[2];
  point[2] = point[1];
  point[1] = -1 * temp;


  return point;
}

// Get area of triangle
// https://en.wikipedia.org/wiki/Area_of_a_triangle#Using_vectors
function area(pointA,pointB,pointC) {
  let ABdotAB=0,ABdotAC=0,ACdotAC=0
  for(let dim = 0; dim < pointA.length; dim++) {
    ABdotAB+=(pointB[dim]-pointA[dim])**2
    ABdotAC+=(pointB[dim]-pointA[dim])*(pointC[dim]-pointA[dim])
    ACdotAC+=(pointC[dim]-pointA[dim])**2
  }
  return Math.sqrt(ABdotAB*ACdotAC - ABdotAC**2)/2
}