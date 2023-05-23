I have a triangle mesh from an IFCJS file. I can take its vertices, its planes, or sample points from its planes.
The mesh provides bounding box.
I have a potree pointcloud. It is separated into depths, and I can take one depth at a time.
The pointcloud provides spacing (min distance between points) at each depth.
I need to align them. They may not have the same scale. They are already coarsely aligned.

I can use ICP with scale to align them.
Problem: 
The triangle mesh contains interior hallways, and the pointcloud contains neighboring buildings, so neither is really a subset of the other.
Normally, I think that needs to be the case for ICP to work well.

We need to exclude the interior hallways or the surrounding buildings.
Could assume that points in pointcloud outside X times the mesh's bounding box at the initial coarse alignment must be bad points.
Could assume that points in pointcloud more than Y times pointcloud's spacing away from the mesh must be bad points, and vice versa.
Could ask user to exclude the part of the pointcloud that is irrelevant.