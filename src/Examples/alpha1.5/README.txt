==================================================================
ALPHA PUZZLE MODEL 
Provided by Boris Yamrom, GE Corporate Research & Development Center

The alpha puzzle benchmark is a motion planning problem containing a 
narrow passage. The puzzle consists of two tubes, each twisted into 
an alpha shape; one tube is the obstacle and the other the moving 
object (robot). The objective is to separate the intertwined tubes.

In order for the problem to be solved a complex set of translation 
and orientation movements need to take place within the narrow 
passage. Computationally the narrow passage needs to be adequately 
mapped which is a difficult problem as this valid space is fairly 
difficult to generate samples in.

Each tube consists of 1008 triangles. The models are given in world 
coordinates in which the `robot' and `obstacle' tubes are in an 
intertwined configuration.

The different versions of this problem were obtained by scaling the 
obstacle tube along the z-axis by a constant factor greater than 1, 
which had the effect of widening the gap between the two prongs of 
the alpha. Thus, the hardest version is the original problem, and 
the easiest version is the 1.5 scaled version. We have solved the 
1.0 version.
==================================================================

File Formats: 
==============
.g/.obj files: BYU FORMAT OBJECT GEOMETRY FILES 
  - one file per object 
  - describe object geometries (in local coordinates)
.env file: ENVIRONMENT FILE 
  - locations of objects in world coordinates
  - identifies "robot" and "obstacle" objects
  - connection information for articulated models
.query file: QUERY FILE
  - gives start and goal configurations for robot 

The BYU and .env formats are described at:
https://parasollab.web.illinois.edu/resources/mpbenchmarks/formats.php

Files: 
==============
README.txt
robot.obj
obstacle.obj
alpha.env
alpha.query
alpha1.5.jpeg

==================================================================
