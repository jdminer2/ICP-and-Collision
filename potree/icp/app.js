import {IfcAPI, IFCCARTESIANPOINT, IFCPRODUCTDEFINITIONSHAPE} from "web-ifc/web-ifc-api";
import {IFCLoader} from "web-ifc-three/IFCLoader"
import {AmbientLight, DirectionalLight, Matrix4, Object3D} from "three";
import { MultiScaleICP } from "./icp";
import {Matrix} from "ml-matrix"

let potreeFilePath = "./0611/metadata.json";;
let ifcFilePath = "mccormick-0416-research.ifc";
let initialGuess = new Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]);

async function run() {
    let scene = loadViewer()
    let pointcloud = await loadPotree(potreeFilePath)
    loadIFC(ifcFilePath,scene,(ifcModel)=>{
        MultiScaleICP(ifcModel,pointcloud,initialGuess).then(transformationMatrix=>
            applyICPResult(pointcloud,transformationMatrix)
        );
        console.log(scene)
    });
    window.removeEventListener("load",run);
}
window.addEventListener("load",run);


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
        $("#menu_appearance").next().show();
    });
    return viewer.scene.scene
}

async function loadPotree(filePath) {
    return await Potree.loadPointCloud(filePath, "pointCloud").then(e => {
        pointcloud=e.pointcloud;
        viewer.scene.addPointCloud(pointcloud);
        
        let material = pointcloud.material;
        material.pointSizeType = Potree.PointSizeType.ADAPTIVE;
        viewer.setEDLOpacity(0.5)
        viewer.fitToScreen();
        viewer.setControls(viewer.fpControls);

        return e.pointcloud
    });
}

async function loadIFC(filePath,scene,onLoad) {
    let ifcLoader = new IFCLoader();
    ifcLoader.load(filePath, (ifcModel) => {
        // Add appropriately-sized ifc model
        let ifcjsContainer = new Object3D();
        ifcjsContainer.rotation.x = Math.PI/2;
        scene.add(ifcjsContainer);
        
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