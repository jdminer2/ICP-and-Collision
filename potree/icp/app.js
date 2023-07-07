import {IFCLoader} from "web-ifc-three/IFCLoader"
import {AmbientLight, DirectionalLight, Matrix4, Object3D, Vector3, Box3, Euler, NormalBlending} from "three";
import { MultiScaleICP, TestOptimalTransformationForIteration } from "./icp";
import { VolumeTool } from "../src/utils/VolumeTool";
import { BoxVolume } from "../src/utils/Volume";
import { PointCloudOctree, PointCloudOctreeNode } from "../src/PointCloudOctree";
import { waitForDef } from "./utils";

let potreeFilePath = "./0611/metadata.json";;
let ifcFilePath = "mccormick-0416-research.ifc";

// Generate the transformation matrix corresponding to applying xRot about the x axis, then yRot about the newly shifted y axis,
// then zRot about the newly shifted z axis, then scale, then xMove, yMove, zMove in the original coordinate space.
//
// Negative scales are not advised for ICP testing, because negative scales mirror the object and turn right-handed into left-handed,
// and this ICP cannot go to negative scales or mirror the object back.
function generateTransformationMatrix(xRot,yRot,zRot,scale,xMove,yMove,zMove) {
    let rotMatrix = new Matrix4().makeRotationFromEuler(new Euler(xRot,yRot,zRot));
    let scaleMatrix = new Matrix4().makeScale(scale,scale,scale);
    let moveMatrix = new Matrix4().makeTranslation(xMove,yMove,zMove);

    return moveMatrix.multiply(scaleMatrix.multiply(rotMatrix))
}

// To uniformly range across all possible rotations, use maxXRot pi/2, maxYRot pi, and maxZRot pi.
// maxScaleFactor should be greater than 0, or else very tiny chance of divide-by-zero, and small chance size will be very large.
function randomTransformationMatrix(maxXRot,maxYRot,maxZRot,maxScaleFactor,maxXMove,maxYMove,maxZMove) {
    // Returns a random value between A and B. It doesn't matter whether A or B is greater.
    function randRange (A,B) {
        let x = Math.random();
        return x*A + (1-x)*B;
    }
    let xRot = randRange(-maxXRot,maxXRot);
    let yRot = randRange(-maxYRot,maxYRot);
    let zRot = randRange(-maxZRot,maxZRot);
    let scale = randRange(1,maxScaleFactor);
    // 50/50 chance of either multiplying or dividing by the selected scale.
    if(Math.random() > 0.5)
        scale = 1/scale;
    let xMove = randRange(-maxXMove,maxXMove);
    let yMove = randRange(-maxYMove,maxYMove);
    let zMove = randRange(-maxZMove,maxZMove);

    console.log(xRot,yRot,zRot,scale,xMove,yMove,zMove);
    return generateTransformationMatrix(xRot,yRot,zRot,scale,xMove,yMove,zMove);
}

let initialGuess;
initialGuess = generateTransformationMatrix(0,0,0,1,0,0,0);
initialGuess = randomTransformationMatrix(0.1,0.1,0.1,1.1,10,10,10);
initialGuess = generateTransformationMatrix(-0.096187,0.074300,-0.024604,1.034383,-0.498889,-8.420134,-9.227480)

async function initialize() {
    loadViewer();
    TestOptimalTransformationForIteration();
    [viewer.schematic, viewer.pointcloud] = await Promise.all([
        loadIFC(ifcFilePath),
        loadPotree(potreeFilePath)
    ]);
    console.log(viewer.schematic);
    console.log(viewer.pointcloud);
    console.log(viewer);
    [viewer.clipVolume, viewer.pclOffset] = createVolumeTools(viewer.schematic, viewer.pointcloud);
    window.removeEventListener("load",initialize);
    viewer.pointcloud.applyMatrix4(initialGuess);
    $('#runICPButton').on("click", (e)=>{
        MultiScaleICP(viewer.schematic,viewer.pointcloud,generateTransformationMatrix(0,0,0,1,0,0,0),viewer.clipVolume).then(transformationMatrix=>
            {}//applyICPResult(viewer.pointcloud,transformationMatrix)
        );
    });    
}
window.addEventListener("load",initialize);



function createVolumeTools(schematic, pointcloud) {
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

    // Create movement tool for the schematic.
    schematic.geometry.computeBoundingBox();
    let schemBoundingBox = schematic.geometry.boundingBox.clone();
    let temp = [schemBoundingBox.min.y,schemBoundingBox.max.y];
    schemBoundingBox.min.y = -1 * schemBoundingBox.max.z;
    schemBoundingBox.max.y = -1 * schemBoundingBox.min.z;
    schemBoundingBox.min.z = temp[0];
    schemBoundingBox.max.z = temp[1];
    let schemPosition = schematic.position;
    let schemVolume = new Potree.BoxVolume();
    schemVolume.scale.copy(schemBoundingBox.max.clone().sub(schemBoundingBox.min));
    schemVolume.position.copy(schemBoundingBox.max.clone().add(schemBoundingBox.min).divideScalar(2).add(schemPosition));
    schemVolume.name = "Schem Bounding Box";
    schemVolume.visible = false;
    viewer.scene.addVolume(schemVolume);

    // Create movement tool for the pointcloud.
    let pclVolume = new Potree.BoxVolume();
    let pclBoundingBox = pointcloud.boundingBox;
    let pclPosition = pointcloud.position
    pclVolume.scale.copy(pclBoundingBox.max.clone().sub(pclBoundingBox.min));
    pclVolume.position.copy(pclBoundingBox.max.clone().add(pclBoundingBox.min).divideScalar(2).add(pclPosition));
    pclVolume.name = "Pcl Bounding Box";
    pclVolume.visible = false;
    viewer.scene.addVolume(pclVolume);

    // Create movement tool for the clipping box.
    let clipVolume  = new Potree.BoxVolume();
    clipVolume.scale.copy(schemVolume.scale.clone().multiply(new Vector3(1, 1, 1)));
    clipVolume.position.copy(schemVolume.position);
    clipVolume.name = "Pcl Cropping Box";
    clipVolume.clip = true;
    viewer.scene.addVolume(clipVolume);

    /* 
        There are 3 sets of coordinates:
        1. The pointcloud's position, rotation, and scale.
        2. The bounding box's position, rotation, and scale.
        3. The user input position, rotation, and scale.
        It will not be possible to have all of these equal. These parameters will help.
        
        userTextboxDiscrepancy: boundingBoxCenter is not at (0,0,0) initially, 
        but user textboxes for position probably should be at (0,0,0) initially. This accounts for that
        pointcloudDiscrepancy: difference between pointcloud center and boundingbox center. 
        This vector will be affected by rotation and scale transformations applied to the pointcloud.
        scaleFactor: when user textbox's scale is (1,1,1), then pointcloud's scale will be (1,1,1), 
        but bounding box must be larger to fit around the pointcloud. This accounts for that.
        
        bounding box's position = user textbox position + userTextboxDiscrepancy
        pointcloud's position = bounding box's position + (pointcloudDiscrepancy with pointcloud's rotation and scale applied)
        pointcloud's scale = user textbox scale
        bounding box's scale = pointcloud's scale * scaleFactor
    */
    let pclOffset = {
        userTextboxDiscrepancy: pclVolume.position.clone(),
        pointcloudDiscrepancy: pointcloud.position.clone().sub(pclVolume.position),
        scaleFactor: pclVolume.scale.clone()
    };
    
    return [clipVolume, pclOffset];
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
}

async function loadPotree(filePath) {
    return await Potree.loadPointCloud(filePath, "pointCloud").then(e => {
        e.pointcloud.name = "Pointcloud";
        viewer.scene.addPointCloud(e.pointcloud);
        
        let material = e.pointcloud.material;
        material.pointSizeType = Potree.PointSizeType.ADAPTIVE;
        viewer.fitToScreen();
        viewer.setControls(viewer.fpControls);

        return e.pointcloud;
    });
}

async function loadIFC(filePath) {
    let ifcLoader = new IFCLoader();
    let schematic;
    ifcLoader.load(filePath, (ifcModel) => {     
        // Some adjustments: name and transparency.   
        ifcModel.name = "Schematic";
        ifcModel.material.forEach(material=>
            material.transparent = true
        )
        
        // Insert into the scene. 
        let ifcjsContainer = new Object3D();
        ifcjsContainer.rotation.x = Math.PI/2; // Because of the difference between IFCJS and Potree viewers.
        ifcjsContainer.add(ifcModel);
        viewer.scene.scene.add(ifcjsContainer);

        // Lighting
        const ambientLight = new AmbientLight(0xffffff, 0.5);
        const directionalLight = new DirectionalLight(0xffffff, 1);
        directionalLight.position.set(0, 10, 0);
        directionalLight.target.position.set(-5, 0, 0);
        ifcjsContainer.add(ambientLight);
        ifcjsContainer.add(directionalLight);
        ifcjsContainer.add(directionalLight.target);

        schematic = ifcModel;
    });
    await waitForDef(()=>schematic?.geometry?.boundingSphere);
    return schematic;
}

function applyICPResult(pointcloud,transformationMatrix) {
    let flattenedTransMatrix = transformationMatrix.data.flatMap(row=>[...row])
    if(Number.isNaN(flattenedTransMatrix[0])) {
        console.log("Failure.");
        return;
    }
    let THREETransMatrix = new Matrix4().set(...flattenedTransMatrix)
    pointcloud.applyMatrix4(THREETransMatrix)
    console.log("Reached");
}