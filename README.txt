Most of the code is a copy from potree. 

New code is located in potree/ICP

To change which files are used:
For pointcloud:
- hierarchy.bin, metadata.json and octree.bin must be in the same folder.
- In app.js, change pointcloudFilePath to point to the metadata.json file.
- I may be unaware of special potree configurations that are exceptions to these instructions. 
  All that matters is that Potree.loadPointCloud(pointcloudFilePath) succeeds.

For schematic:
- In app.js, change schematicFilePath to point to the .ifc or .obj file.

To run it:

1. Set the file references as above.
2. Open the the project in terminal and navigate to the potree folder.
> npm install
> npm start
3. Open localhost:1234/icp
