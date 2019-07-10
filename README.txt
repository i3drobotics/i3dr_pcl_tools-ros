DESCRIPTION
-------------
VREP simulation of Deimos and Phobos cameras.
Phobos camera is attached to the UR10.
Deimos camera is attached to the UARM.
Simulation publishes the images to a ROS node.
Images can be viewed using the provided 'camera_viewer' node
Images can be saved to file using the provided 'camera_saver' node

FILES
-------
/doc/models: urdf and stl models of phobos and deimos (used in V-REP)
/doc/sim/sim.ttt: V-REP simulation file
/src/camera_viewer.cpp: C++ phobos and deimos camera image viewer node
/src/camera_saver.cpp: C++ phobos and deimos camera image saver node

DEPENDANCES
-----------
libopencv-dev
cv_bridge

SETUP
------
Add this package to your ros workspace in /src/ folder
Build with cmake_build

RUN
-----
Start simulation:
	in new terminal: roscore
	open V-REP
	load simulation: doc/simulations/deimos_phobos/deimos_phobos.ttt
	(NOTE: roscore must run first before opening v-rep for it to work with ROS)
View deimos and phobos camera images:
	in new terminal: rosrun vrep-sim camera_viewer
Save deimos and phobos camera images to file:
	in new terminal: rosrun vrep-sim camera_saver
	example: rosrun vrep-sim camera_saver img_capture/
	images are saved in the format [cameraName_frameNum_LR.jpg]
	(NOTE: Large amount of images will be captured quickly)
