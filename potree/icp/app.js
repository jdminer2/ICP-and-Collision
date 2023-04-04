import {IfcAPI, IFCCARTESIANPOINT} from "web-ifc/web-ifc-api";
import {IFCLoader} from "web-ifc-three/IFCLoader"
import {AmbientLight, DirectionalLight, Matrix4, Object3D} from "three";
import { MultiScaleICP } from "./icp";
import {Matrix} from "ml-matrix"

let potreeFilePath = "./0611/metadata.json"
let ifcjsFilePath = "mccormick-0416-research.ifc"

async function activate() {
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
        //viewer.toggleSidebar();
    });
    let pointcloud;
    await Potree.loadPointCloud(potreeFilePath, "pointCloud").then(e => {
        pointcloud=e.pointcloud;
        viewer.scene.addPointCloud(pointcloud);
        
        let material = pointcloud.material;
        material.pointSizeType = Potree.PointSizeType.ADAPTIVE;
        viewer.fitToScreen();
        viewer.setControls(viewer.fpControls);
    });


    // IFCJS loading
    let modelID = 0;

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
    await ifcapi.Init().then(()=>getIfcFile(ifcjsFilePath)).then((ifcData) => {
        modelID = ifcapi.OpenModel(ifcData);
        let lines = ifcapi.GetLineIDsWithType(modelID,IFCCARTESIANPOINT);
        
        for(let i = 0; i < lines.size(); i++)
            ifcPoints.push(ifcapi.GetLine(modelID,lines.get(i)));
    });

    // IFCJS rendering
    let ifcLoader = new IFCLoader();
    ifcLoader.load(ifcjsFilePath, (ifcModel) => {
        // Add appropriately-sized ifc model
        let ifcjsContainer = new Object3D();
        viewer.scene.scene.add(ifcjsContainer);
        
        ifcjsContainer.rotation.x = Math.PI/2;

        ifcjsContainer.add(ifcModel);


        // Lighting
        const lightColor = 0xffffff;

        const ambientLight = new AmbientLight(lightColor, 0.5);
        ifcjsContainer.add(ambientLight);

        const directionalLight = new DirectionalLight(lightColor, 1);
        directionalLight.position.set(0, 10, 0);
        directionalLight.target.position.set(-5, 0, 0);
        ifcjsContainer.add(directionalLight);
        ifcjsContainer.add(directionalLight.target);
    });

    MultiScaleICP(ifcPoints,pointcloud,new Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])).then(transformationMatrix => {
        let flattenedTransMatrix = transformationMatrix.data.flatMap(row=>[...row])
        let THREETransMatrix = new Matrix4().set(...flattenedTransMatrix)
        pointcloud.applyMatrix4(THREETransMatrix.invert())
        console.log(...transformationMatrix.data)
    });

    window.removeEventListener("load",activate);
}
window.addEventListener("load",activate);