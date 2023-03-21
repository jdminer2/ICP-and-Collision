import {IfcAPI, IFCCARTESIANPOINT} from "web-ifc/web-ifc-api";
import {IFCLoader} from "web-ifc-three/IFCLoader"
import {Object3D, AmbientLight, DirectionalLight} from "three";

let potreeFilePath = "./0611/metadata.json"
let ifcjsFilePath = "mccormick-0416-research.ifc"

function activate() {
    // Potree
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

    Potree.loadPointCloud(potreeFilePath, "pointCloud", function(e){
        viewer.scene.addPointCloud(e.pointcloud);
        
        let material = e.pointcloud.material;
        material.pointSizeType = Potree.PointSizeType.ADAPTIVE;
        viewer.fitToScreen();
        viewer.setControls(viewer.fpControls);

        console.log(e);
    });


    // IFCJS
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
        getIfcFile(ifcjsFilePath).then((ifcData) => {
            modelID = ifcapi.OpenModel(ifcData);
            let lines = ifcapi.GetLineIDsWithType(modelID,IFCCARTESIANPOINT);
            
            let cartesianPoints = [];
            for(let i = 0; i < lines.size(); i++)
                cartesianPoints.push(ifcapi.GetLine(modelID,lines.get(i)));
            console.log("IFCJS CARTESIANPOINTS: ", cartesianPoints);
        });
    });

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
    window.removeEventListener("load",activate);
}
window.addEventListener("load",activate);