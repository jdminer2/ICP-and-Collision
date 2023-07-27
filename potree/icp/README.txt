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


Could do kd-tree nearest neighbor search on pointcloud, which is helpful for feature detection on pointcloud. 
This identifies the points that are on sharp edges and corners. https://drops.dagstuhl.de/opus/volltexte/2011/3101/pdf/12.pdf

It may be hard to identify the sharp edges and corners of the IFC mesh, because it splits polygons into triangles. 
To reverse this: For each edge of the mesh: it is a feature iff there are not exactly 2 coplanar triangles meeting at the edge.

Once I have identified the sharp edges and corners of both graphs, 



https://github.com/XuyangBai/awesome-point-cloud-registration


Use one of the registration methods:
https://github.com/prs-eth/OverlapPredator MIT license (good) this is in python. It seems very good.
Open3D with the multiscaleICP is in python or c++.

However, potree and ifcjs are in javascript.

https://tms-dev-blog.com/python-backend-with-javascript-frontend-how-to/

I need to figure out how to make a project with both python and javascript, preferably in a way that allows easy swapping-out of different python solutions.
I want to just put the points into the python code, and get the transformation out.

I need to fix my potree accessor stuff to get rid of the black pixel bug.
It's possible potree has non-javascript accessor stuff which could be faster.

AI pre-trained models provided as solutions were probably only trained on things that were honestly matching. 
These buildings are at different stages of constructions, so they are not honestly matching; they are differently shaped but similar objects. 
I don't know if the pre-trained models will work on not-honest-matches, but they might.

I also want the algorithm to use the initial guess, because we humans know the context (neighboring buildings and colors), but the algorithm does not.
Very similar buildings or rotationally symmetric buildings or buildings where one of the 3 dimensions does not have an honestly matching surface 
(so you could translate in that dimension) would probably all cause problems and require the initial guess. 
The clipping volume might partially solve the issue but not completely.