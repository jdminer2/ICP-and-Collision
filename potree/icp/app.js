import {IFCLoader} from "web-ifc-three/IFCLoader"
import {AmbientLight, DirectionalLight, Matrix4, Object3D, Vector3, Euler} from "three";
import { multiScaleICP, testGetBestTransformation } from "./icp";
import { waitForDef } from "./utils";
import { OBJLoader } from "../libs/three.js/loaders/OBJLoader.js";

// Must point to the metadata.json file for the pointcloud, with related files nearby as described in the README.
//let potreeFilePath = "./geometryFiles/0611File/metadata.json";
let potreeFilePath = "./geometryFiles/jointFile/metadata.json"; 
// Schematic must end in .ifc or .obj
//let schematicFilePath = "./geometryFiles/0611File/mccormick-0416-research.ifc";
let schematicFilePath = "./geometryFiles/jointFile/joint_model.obj";

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
//initialGuess = randomTransformationMatrix(0.1,0.1,0.1,1.1,10,10,10);
//initialGuess = generateTransformationMatrix(-0.096187,0.074300,-0.024604,1.034383,-0.498889,-8.420134,-9.227480)

async function initialize() {
    loadViewer();
    testGetBestTransformation();
    [viewer.schematic, viewer.pointcloud] = await Promise.all([
        loadSchematic(schematicFilePath),
        loadPotree(potreeFilePath)
    ]);
    console.log(viewer.schematic);
    console.log(viewer.pointcloud);
    console.log(viewer);
    [viewer.clipVolume, viewer.pclOffset] = createVolumeTools(viewer.schematic, viewer.pointcloud);
    window.removeEventListener("load",initialize);
    viewer.pointcloud.applyMatrix4(initialGuess);
    $('#runICPButton').on("click", (e)=>{
        multiScaleICP(viewer.schematic,viewer.pointcloud,viewer.clipVolume).then(transformationMatrix=>
            {}
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

async function loadSchematic(filePath) {
    // Check if it's IFC or OBJ file.
    let filePathExtension = filePath.split('.').pop();
    if(!filePathExtension)
        throw new Error("Unsupported: schematic file extension missing. Only supports .ifc and .obj currently.");
    if(!["ifc","obj"].includes(filePathExtension.toLowerCase()))
        throw new Error("Unsupported schematic file extension: " + filePathExtension + ". Only supports .ifc and .obj currently.");
    let isIFC = "ifc" === filePathExtension.toLowerCase();

    let loader = isIFC ? new IFCLoader() : new OBJLoader();
    let schematic;
    loader.load(filePath, model => {
        schematic = isIFC ? model : model.children[0];
        schematic.name = "Schematic";
        schematic.isIFC = isIFC;

        // Material transparency enabled for the sidebar slider.
        if(Array.isArray(schematic.material))
            schematic.material.forEach(material=>
                material.transparent = true
            )
        else
            schematic.material.transparent = true;
        
        // Insert into the scene. 
        let schematicContainer = new Object3D();
        // Because Potree seems to be rotated Math.PI compared to other things.
        schematicContainer.rotation.x = Math.PI/2;
        schematicContainer.add(schematic);

        // Add lighting
        const ambientLight = new AmbientLight(0xffffff, 0.5);
        const directionalLight = new DirectionalLight(0xffffff, 1);
        directionalLight.position.set(0, 10, 0);
        directionalLight.target.position.set(-5, 0, 0);
        schematicContainer.add(ambientLight);
        schematicContainer.add(directionalLight);
        schematicContainer.add(directionalLight.target);

        viewer.scene.scene.add(schematicContainer);
    });
    await waitForDef(()=>schematic?.geometry?.boundingSphere);
    return schematic;
}