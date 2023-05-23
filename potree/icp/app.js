import {IFCLoader} from "web-ifc-three/IFCLoader"
import {AmbientLight, DirectionalLight, Matrix4, Object3D, Box3, NormalBlending} from "three";
import { MultiScaleICP } from "./icp";
import {Matrix} from "ml-matrix"
import { VolumeTool } from "../src/utils/VolumeTool";
import { BoxVolume } from "../src/utils/Volume";
import { PointCloudOctree, PointCloudOctreeNode } from "../src/PointCloudOctree";

let potreeFilePath = "./0611/metadata.json";;
let ifcFilePath = "mccormick-0416-research.ifc";
let initialGuess = new Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]);

async function run() {
    loadViewer()
    let pointcloud = await loadPotree(potreeFilePath)
    loadIFC(ifcFilePath,async (ifcModel)=>{
        console.log(ifcModel)
        console.log(pointcloud);
        console.log(viewer)
        ifcModel.material.forEach(material=>{
            material.transparent = true;
            material.opacity = 0.5;
        });
        await waitForDef(()=>ifcModel?.geometry?.boundingSphere)
        BoundPotree(ifcModel,pointcloud);
        let clippingVolume = await waitForDef(()=> {
            let clippingVolume;
            viewer.inputHandler.scene.volumes.forEach(volume => {
                if(volume.name === "Pcl Cropping Box")
                    clippingVolume = volume;
            });
            return clippingVolume;
        });
        let node = new PointCloudOctreeNode();
        node.geometryNode = pointcloud.root.geometryNode;
        node.sceneNode = pointcloud.root.sceneNode;
        await waitForDef(()=>pointcloud.root.getPointsInBox.bind(pointcloud.root));
        let root = pointcloud.root;
        console.log(root.getPointsInBox(clippingVolume));
        MultiScaleICP(ifcFilePath,ifcModel,pointcloud,initialGuess,5).then(transformationMatrix=>
            applyICPResult(pointcloud,transformationMatrix)
        );
    });
    window.removeEventListener("load",run);
}
window.addEventListener("load",run);

function BoundPotree(ifcModel,pointcloud) {
    ifcModel.geometry.computeBoundingBox();
    let schemBoundingBox = ifcModel.geometry.boundingBox.clone();
    let temp = [schemBoundingBox.min.y,schemBoundingBox.max.y];
    schemBoundingBox.min.y = -1 * schemBoundingBox.max.z;
    schemBoundingBox.max.y = -1 * schemBoundingBox.min.z;
    schemBoundingBox.min.z = temp[0];
    schemBoundingBox.max.z = temp[1];

    let schemPosition = ifcModel.position;
    let pclBoundingBox = pointcloud.boundingBox;
    let pclPosition = pointcloud.position;

    function scaleCoords(boundingBox){
        return ['x','y','z'].map(dim=>
            boundingBox.max[dim] - boundingBox.min[dim]
        )
    }
    function positionCoords(boundingBox,position){
        return ['x','y','z'].map(dim=>
            (boundingBox.max[dim] + boundingBox.min[dim])/2 + position[dim]
        )
    }

    let schemVolume = new Potree.BoxVolume();
    schemVolume.name = "Schem Bounding Box";
    schemVolume.scale.set(...scaleCoords(schemBoundingBox));
    schemVolume.position.set(...positionCoords(schemBoundingBox,schemPosition))
    schemVolume.visible = true;

    let pclVolume = new Potree.BoxVolume();
    pclVolume.name = "Pcl Bounding Box";
    pclVolume.scale.set(...scaleCoords(pclBoundingBox));
    pclVolume.position.set(...positionCoords(pclBoundingBox,pclPosition));
    pclVolume.visible = false;

    let clipVolume  = new Potree.BoxVolume();
    clipVolume.name = "Pcl Cropping Box";
    clipVolume.scale.set(...scaleCoords(schemBoundingBox));
    clipVolume.scale.x *= 1.1;
    clipVolume.scale.y *= 1.1;
    clipVolume.scale.z *= 2.5;
    clipVolume.position.set(...positionCoords(schemBoundingBox,schemPosition));
    clipVolume.clip = true;
    viewer.scene.addVolume(schemVolume);
    viewer.scene.addVolume(pclVolume);
    viewer.scene.addVolume(clipVolume);
}


function loadViewer() {
    // Potree loading and rendering
    window.viewer = new Potree.Viewer(document.getElementById("potree_render_area"));
    viewer.setEDLEnabled(true);
    viewer.setFOV(60);
    viewer.setPointBudget(1_000_000);
    viewer.loadSettingsFromURL();
    viewer.setDescription("Loading Octree of LAS files");
    viewer.loadGUI(() => {
        viewer.setLanguage('en');
        $("#menu_icp").next().show();
        $("#menu_appearance").next().show();
        $("#menu_tools").next().show();
        viewer.toggleSidebar();
    });
    return viewer.scene.scene
}

async function loadPotree(filePath) {
    return await Potree.loadPointCloud(filePath, "pointCloud").then(e => {
        pointcloud=e.pointcloud;
        pointcloud.name = "Pointcloud";
        viewer.scene.addPointCloud(pointcloud);
        
        let material = pointcloud.material;
        material.pointSizeType = Potree.PointSizeType.ADAPTIVE;
        viewer.fitToScreen();
        viewer.setControls(viewer.fpControls);

        return e.pointcloud
    });
}

async function loadIFC(filePath,onLoad) {
    let ifcLoader = new IFCLoader();
    ifcLoader.load(filePath, (ifcModel) => {
        // Add appropriately-sized ifc model
        let ifcjsContainer = new Object3D();
        ifcjsContainer.rotation.x = Math.PI/2;
        viewer.scene.scene.add(ifcjsContainer);
        
        ifcModel.name = "Schematic";
        ifcjsContainer.add(ifcModel);

        // Lighting
        const ambientLight = new AmbientLight(0xffffff, 0.5);
        ifcjsContainer.add(ambientLight);

        const directionalLight = new DirectionalLight(0xffffff, 1);
        directionalLight.position.set(0, 10, 0);
        directionalLight.target.position.set(-5, 0, 0);
        ifcjsContainer.add(directionalLight);
        ifcjsContainer.add(directionalLight.target);

        onLoad(ifcModel)
    });
}

function applyICPResult(pointcloud,transformationMatrix) {
    let flattenedTransMatrix = transformationMatrix.data.flatMap(row=>[...row])
    let THREETransMatrix = new Matrix4().set(...flattenedTransMatrix)
    pointcloud.applyMatrix4(THREETransMatrix.invert())
}

async function waitForDef(getter) {
    let val = getter();
    while(val === undefined || val === null) {
      await new Promise(e=>setTimeout(e,0));
      val = getter();
    }
    return val
  }