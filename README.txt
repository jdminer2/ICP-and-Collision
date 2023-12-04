Most of the code is a copy from potree. 

New code is located in potree/ICP

To change which files are used:
For pointcloud:
- In app.js, change pointcloudFilePath to point to the metadata.json file of the pointcloud.
- Expects hierarchy.bin, octree.bin, and data/<FILE> to be in the same folder as metadata.json,
  where <FILE> is the .las or other file type supported by Potree. <FILE>'s name should match the "name" field in metadata.json.
- There are other file structures loadable by Potree, but I am unfamiliar with them.
  All that matters is that Potree.loadPointCloud(pointcloudFilePath) succeeds.

For schematic:
- In app.js, change schematicFilePath to point to the .ifc or .obj file.

To run it:

1. Set the file references as above.
2. Open the the project in terminal and navigate to the potree folder.
> npm install
> npm start
3. Open localhost:1234/icp
