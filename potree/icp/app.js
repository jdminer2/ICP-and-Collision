import {IFCLoader} from "web-ifc-three/IFCLoader"
import {AmbientLight, DirectionalLight, Matrix4, Object3D, Vector3, Euler} from "three";
import { multiScaleICP, testGetBestTransformation } from "./icp";
import { initializePclCropTextboxes, updatePointcloudTextboxes, updateSchematicTextboxes, waitForDef } from "./utils";
import { OBJLoader } from "../libs/three.js/loaders/OBJLoader.js";

// Must point to the metadata.json file for the pointcloud, with related files nearby as described in the README.
//let pointcloudFilePath = "./geometryFiles/0611File/0611/metadata.json";
let pointcloudFilePath = "./geometryFiles/jointFile/joint/metadata.json"; 
// Schematic must end in .ifc or .obj
//let schematicFilePath = "./geometryFiles/0611File/mccormick-0416-research.ifc";
let schematicFilePath = "./geometryFiles/jointFile/joint_model.obj";

// Generate the transformation matrix corresponding to applying xRot about the x axis, then yRot about the newly shifted y axis,
// then zRot about the newly shifted z axis, then scale, then xMove, yMove, zMove in the original coordinate space.
//
// Negative scales are not advised for ICP testing, because negative scales mirror the object and turn right-handed into left-handed,
// and this ICP cannot go to negative scales or mirror the object back.
function generateTransformationMatrix(xMove,yMove,zMove,xRot,yRot,zRot,scale) {
    let moveMatrix = new Matrix4().makeTranslation(xMove,yMove,zMove);
    let rotMatrix = new Matrix4().makeRotationFromEuler(new Euler(xRot,yRot,zRot));
    let scaleMatrix = new Matrix4().makeScale(scale,scale,scale);

    return moveMatrix.multiply(scaleMatrix.multiply(rotMatrix))
}

// To uniformly range across all possible rotations, use maxXRot pi/2, maxYRot pi, and maxZRot pi.
// maxScaleFactor should be greater than 0, or else very tiny chance of divide-by-zero, and small chance size will be very large.
function randomTransformation(maxXMove,maxYMove,maxZMove,maxXRot,maxYRot,maxZRot,maxScaleFactor) {
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

    console.log(xMove,yMove,zMove,xRot,yRot,zRot,scale);
    return [xMove,yMove,zMove,xRot,yRot,zRot,scale];
}

async function initialize() {
    window.removeEventListener("load",initialize);

    // Setup viewer, load schematic and pointcloud from file.
    setupViewer();
    [viewer.schematic, viewer.pointcloud] = await Promise.all([
        loadSchematic(schematicFilePath),
        loadPointcloud(pointcloudFilePath)
    ]);

    // Rotate the pointcloud (testing purposes)
    // let initialMatrix = randomTransformation(10,10,10,0.1,0.1,0.1,1.1);
    // let initialMatrix = [-0.498889,-8.420134,-9.227480,-0.096187,0.074300,-0.024604,1.034383];
    // pointcloud.applyMatrix4(initialMatrix);

    // Create transformation tools.
    createTransformationTools(viewer.schematic, viewer.pointcloud);

    // Update textbox values.
    updateSchematicTextboxes();
    updatePointcloudTextboxes();
    initializePclCropTextboxes();

    // Tests whether ICP's single-iteration transformation-finding function is optimal.
    // testGetBestTransformation();
    
    // Make event listener for ICP button.
    $('#runICPButton').on("click", (e)=>{
        multiScaleICP(viewer.schematic,viewer.pointcloud,viewer.clipVolume).then(transformationMatrix=>{
            updatePointcloudTextboxes();
        });
    });
}
window.addEventListener("load",initialize);


function createTransformationTools(schematic, pointcloud) {
    /* 
        For each object (pointcloud and schematic) there are 3 sets of position, rotation, scale coordinates:
        1. The textbox's listed position, rotation, scale.
        2. The volume tool's position, rotation, scale in the scene.
        3. The object's position, rotation, scale in the scene.
        There are some issues with the differences between these.
        
        First, the textbox will only have one scale parameter, for uniform scale changes, 
        even though the object may have different scale values for each dimension. Therefore, a scale factor vector will be tracked.
        Multiply textbox scale by objectScaleFactor to get object scale.

        Second, if an object's scale is small (after scale factor), it may still be very big in the scene,
        because the object's scale does not say anything about the size and positioning of triangles or points belonging to the object.
        Meanwhile, if the volume tool's scale is small like (1,1,1), it will be small in the scene, like a 1x1x1 box.
        This could make volume tool and scale handles very tiny relative to the object, making it hard to use. Therefore, a second scale factor will be tracked.
        Multiply textbox scale by volumeScaleFactor to get volume tool scale.

        Third, rotation and scale will occur about the object's position.
        However, all the triangles or points parented to the object may be far off to the side, 
        so the rotation and scaling may not look like it's about the center of the object.
        It may be better if rotation and scaling happen about the object's center of bounding box. Therefore, a position offset is tracked.
        Add position offset (with rotation and scale applied to it) to object position to get textbox position, volume tool position, and center of rotation/scaling.
        (it is not the internal center of rotation/scaling, but we make adjustments so that inputted rotations and scales seem to occur about it)

        Fourth, the schematic seems to be rotated Pi/2 compared to everything else, causing problems
    */

    // Create schematic volume tool
    let schemVolume = new Potree.BoxVolume();
    viewer.scene.addVolume(schemVolume);
    viewer.schemVolume = schemVolume;
    schemVolume.name = "Schem Bounding Box";
    schemVolume.visible = false;
    schemVolume.position = new Vector3();
    schemVolume.rotation = new Euler();
    schemVolume.scale = new Vector3(1,1,1);

    // Force schematic bounding box to generate.
    viewer.schematic.geometry.computeBoundingBox();
    // Correct differences between scene coordinate space and schematic coordinate space.
    let schemBoundingBox = viewer.schematic.geometry.boundingBox.clone();
    let temp = [schemBoundingBox.min.y,schemBoundingBox.max.y];
    schemBoundingBox.min.y = -1 * schemBoundingBox.max.z;
    schemBoundingBox.max.y = -1 * schemBoundingBox.min.z;
    schemBoundingBox.min.z = temp[0];
    schemBoundingBox.max.z = temp[1];
    // Compute offsets
    schemVolume.objectScaleFactor = schematic.scale.clone();
    schemVolume.volumeScaleFactor = schemBoundingBox.max.clone().sub(schemBoundingBox.min);
    schemVolume.positionOffset = schemBoundingBox.max.clone().add(schemBoundingBox.min).divideScalar(2);

    // Create pointcloud volume tool.
    let pclVolume = new Potree.BoxVolume();
    viewer.scene.addVolume(pclVolume);
    viewer.pclVolume = pclVolume;
    pclVolume.name = "Pcl Bounding Box";
    pclVolume.visible = false;
    pclVolume.position = new Vector3();
    pclVolume.rotation = new Euler();
    pclVolume.scale = new Vector3(1,1,1);

    // Compute offsets.
    pclVolume.objectScaleFactor = pointcloud.scale.clone();
    pclVolume.volumeScaleFactor = pointcloud.boundingBox.max.clone().sub(pointcloud.boundingBox.min);
    pclVolume.positionOffset = pointcloud.boundingBox.max.clone().add(pointcloud.boundingBox.min).divideScalar(2);

    // Create clipping box volume tool.
    let clipVolume  = new Potree.BoxVolume();
    viewer.scene.addVolume(clipVolume);
    viewer.clipVolume = clipVolume;
    clipVolume.name = "Pcl Cropping Box";
    clipVolume.clip = true;
    clipVolume.position = new Vector3();
    clipVolume.rotation = new Euler();
    clipVolume.scale = new Vector3(1,1,1);
}

function setupViewer() {
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

async function loadPointcloud(filePath) {
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