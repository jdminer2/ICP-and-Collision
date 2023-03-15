import {IfcAPI, IFCCARTESIANPOINT} from "web-ifc/web-ifc-api";
import {IFCLoader} from "web-ifc-three/IFCLoader"
import {Object3D, AmbientLight, DirectionalLight} from "THREE";


// Potree
window.viewer = new Potree.Viewer(document.getElementById("potree_render_area"));
document.getElementById("potree_render_area").getElementsByTagName("canvas")[0].id = "potree-canvas";

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

Potree.loadPointCloud("./0611/metadata.json", "lion", function(e){
    viewer.scene.addPointCloud(e.pointcloud);
    
    let material = e.pointcloud.material;
    material.size = 1;
    material.pointSizeType = Potree.PointSizeType.ADAPTIVE;
    
    e.pointcloud.position.x += 3;
    e.pointcloud.position.y -= 3;
    e.pointcloud.position.z += 4;
    
    viewer.fitToScreen();
    console.log(e.pointcloud);
});


// IFCJS
const ifcFileLocation = "mccormick-0416-research.ifc";
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
ifcapi.Init().then(()=>{
    getIfcFile(ifcFileLocation).then((ifcData) => {
        console.log("Reached IFCJS");
        modelID = ifcapi.OpenModel(ifcData);
        let lines = ifcapi.GetLineIDsWithType(modelID,IFCCARTESIANPOINT);
        
        let cartesianPoints = [];
        for(let i = 0; i < lines.size(); i++)
            cartesianPoints.push(ifcapi.GetLine(modelID,lines.get(i)));
        console.log("IFCJS CARTESIANPOINTS: ", cartesianPoints);
    });
});

let ifcLoader = new IFCLoader();
ifcLoader.load(ifcFileLocation, (ifcModel) => {
    // Add appropriately-sized ifc model
    let scene = new Object3D();
    viewer.scene.scene.add(scene);
    
    scene.position.z = 500;
    scene.rotation.x = Math.PI/2;
    scene.scale.x = 25;
    scene.scale.y = 25;
    scene.scale.z = 25;

    scene.add(ifcModel);


    // Lighting
    const lightColor = 0xffffff;

    const ambientLight = new AmbientLight(lightColor, 0.5);
    scene.add(ambientLight);

    const directionalLight = new DirectionalLight(lightColor, 1);
    directionalLight.position.set(0, 10, 0);
    directionalLight.target.position.set(-5, 0, 0);
    scene.add(directionalLight);
    scene.add(directionalLight.target);
});