import { OctreeGeometry, OctreeGeometryNode } from "../src/modules/loader/2.0/OctreeGeometry";
import { BufferGeometry, BufferAttribute } from "../libs/three.js/build/three.module";
import {NodeLoader} from "../src/modules/loader/2.0/OctreeLoader"

  // Asynchronously waits for the value to become defined
  async function waitForDef(getter) {
    let val = getter();
    while(val === undefined) {
      await new Promise(e=>setTimeout(e,0));
      val = getter();
    }
    return val
  }

  // Copies the root of the pointcloud so that loading its children doesn't affect which points are loaded in the rendered scene
  async function getRootNode(pointcloud) {
    return await waitForDef(()=>pointcloud?.pcoGeometry?.root).then(root=>{
      let nodeGeom = {...root.octreeGeometry}
      nodeGeom.loader={...nodeGeom.loader}

      let node = new OctreeGeometryNode("r", nodeGeom, root.boundingBox)
      node.level = root.level;
      node.nodeType = root.nodeType;
      node.hierarchyByteOffset = root.hierarchyByteOffset;
      node.hierarchyByteSize = root.hierarchyByteSize;
      node.hasChildren = root.hasChildren;
      node.spacing = root.spacing;
      node.byteOffset = root.byteOffset;
      node.byteSize = root.byteSize;
      node.numPoints = root.numPoints;
      return node
    })
  }

  // Resolves with all points in the pointcloud from nodes to the specified depth.
  export async function getAllPoints(pointcloud,depth) {
    let nodes = await getAllNodes(pointcloud,depth)
    let points = [];
    let position = pointcloud.position
    nodes.forEach(node=>{
        let array = node.geometry.attributes.position.array;
        let dims = node.geometry.attributes.position.itemSize;
        for(let i = 0; i <= array.length - dims; i += dims)
            points.push([array[i] + position.x, array[i+1] + position.y, array[i+2] + position.z]);
    });
    return points
  }

  // Resolves with all nodes of the pointcloud to the specified depth, with their points loaded and ready to access.
  export async function getAllNodes(pointcloud,depth) {
    let root = await getRootNode(pointcloud)
    return await getAllNodesHelper(root,depth)
  }

  // Resolves with an array containing the node and its descendants to the specified depth, with all their points loaded and ready to access.
  async function getAllNodesHelper(node,depth) {
    loadNode(node)
    let returnArray = [node]
    if(depth > 0) {
        await waitForDef(()=>node.children).then(async children=>{
          let childArray = Object.values(children)
          let promisedChildArray = childArray.map(child=>getAllNodesHelper(child,depth-1))
          returnArray.push(promisedChildArray)
          // Flatten, resolve, and flatten, to remove all the nesting and promises
          returnArray = (await Promise.all(returnArray.flat())).flat()
        })
      }
    await waitForDef(()=>node.geometry?.attributes?.position)
    return returnArray
  }

  // NodeLoader.load, without being blocked by the restriction of how many nodes are loaded, and with some details not accessed
  // This makes points and children available.
  async function loadNode(node) {
    try{
      if(node.nodeType === 2){
        await node.octreeGeometry.loader.loadHierarchy(node);
      }

      let {byteOffset, byteSize} = node;


      let urlOctree = `${node.octreeGeometry.loader.url}/../octree.bin`;

      let first = byteOffset;
      let last = byteOffset + byteSize - 1n;

      let buffer;

      if(byteSize === 0n){
        buffer = new ArrayBuffer(0);
        console.warn(`loaded node with 0 bytes: ${node.name}`);
      }else {
        let response = await fetch(urlOctree, {
          headers: {
            'content-type': 'multipart/byteranges',
            'Range': `bytes=${first}-${last}`,
          },
        });

        buffer = await response.arrayBuffer();
      }

      let workerPath;
      if(node.octreeGeometry.loader.metadata.encoding === "BROTLI"){
        workerPath = Potree.scriptPath + '/workers/2.0/DecoderWorker_brotli.js';
      }else {
        workerPath = Potree.scriptPath + '/workers/2.0/DecoderWorker.js';
      }

      let worker = Potree.workerPool.getWorker(workerPath);

      worker.onmessage = function (e) {

        let data = e.data;
        let buffers = data.attributeBuffers;

        Potree.workerPool.returnWorker(workerPath, worker);

        let geometry = new BufferGeometry();
        
        for(let property in buffers)
          if(property === "position"){
            let buffer = buffers[property].buffer;
            geometry.setAttribute('position', new BufferAttribute(new Float32Array(buffer), 3));
          }

        node.density = data.density;
        node.geometry = geometry;
      };

      let pointAttributes = node.octreeGeometry.pointAttributes;
      let scale = node.octreeGeometry.scale;

      let box = node.boundingBox;
      let min = node.octreeGeometry.offset.clone().add(box.min);
      let size = box.max.clone().sub(box.min);
      let max = min.clone().add(size);
      let numPoints = node.numPoints;

      let offset = node.octreeGeometry.loader.offset;

      let message = {
        name: node.name,
        buffer: buffer,
        pointAttributes: pointAttributes,
        scale: scale,
        min: min,
        max: max,
        size: size,
        offset: offset,
        numPoints: numPoints
      };

      worker.postMessage(message, [message.buffer]);
    }catch(e){
      console.log(`failed to load ${node.name}`);
      console.log(e);
      console.log(`trying again!`);
    }
  }