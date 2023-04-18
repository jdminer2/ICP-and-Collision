  export function getPointsFromModel(ifcModel,pointCount) {
    let points = samplePointsFromTriangles(getSetOfTriangles(ifcModel),pointCount)
    return points
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

  // Almost-uniformly sample specified number of points from given set of triangles
  function samplePointsFromTriangles(setOfTriangles, pointCount) {
    let points = [];
    
    let foundArea = 0;
    const totalArea = setOfTriangles.reduce((sum,triangle)=>sum+area(...triangle),0);
    if(totalArea <= 0)
      return [];

    let pointsDone = 0;
    for(let i = 0; i < setOfTriangles.length; i++) {
      const triangle = setOfTriangles[i];
      foundArea += area(...triangle);

      // Add more points, until adding another point would exceed limit.
      while(pointsDone+1 <= pointCount * Math.min(1,foundArea/totalArea))
        points.push(samplePointFromTriangle(...triangle,pointsDone++));
    }

    // Make sure the final triangle reaches pointCount (in case foundArea didn't reach totalArea due to rounding errors)
    while(pointsDone < pointCount)
      points.push(samplePointFromTriangle(...(setOfTriangles.slice(-1)),pointsDone++));

    return points;
  }

  // Almost-uniformly sample from triangle
  // https://web.archive.org/web/20190606192126/https://extremelearning.com.au/evenly-distributing-points-in-a-triangle/
  function samplePointFromTriangle(pointA,pointB,pointC,idx) {
    // Get a quasirandom point in unit square using quasiR2 sequence
    const plastic = Math.cbrt((9+Math.sqrt(69))/18) + Math.cbrt((9-Math.sqrt(69))/18); // https://en.wikipedia.org/wiki/Plastic_number
    let [x,y] = [((idx+20)/plastic)%1,((idx+20)/plastic**2)%1];
    
    // Transform the point to from unit square to unit triangle
    if(x+y>1)
      [x,y] = [1-x,1-y];
    
    // Relabel given triangle so point A is opposite the longest side
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