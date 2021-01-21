# special_course

Code made by Carla Salazar as part of a special course at DTU. In this course, the infrastructure propose in this repository has been prepared to test the efficiency of different computer vision sensors (cameras or lidars). For this purpose, a special target was designed and built. It looks as follows:

![GitHub Logo](/images/target.png)

With this target and this code, the following information information can be obtained:
  * Distance error to a defined target.
  * Standard deviation of the distance to this target.
  * Distance error to smaller targets of different shapes (5x5, 4x4, 3x3, 3x1, 3x0.5, 3x0.2 cm).
  * Standard deviation of the distance to the smaller targets.
  * Horizontal accurazy of the size of the smaller targets.
  * Vertical accurazy of the size of the smaller targets.
This informacion can be gotten in different environments or with different target materials to analyze how the sensor performs when changing the environment. For example, a wood and metal target can be used to test which sensors seems more affected by the reflectivity of the material.

The code is formed by 3 parts:
  * ROS code for recording data from these possible sensors:
    * Realsense L515
    * Realsense D435i
    * ZED2
    * Livox Mid-40
    * Mynteye D1000
  * Python file that converts all PLY in directory to PCD saving PLYs to another directory.
  * C++ files for processing the PCD files of the recorded targets to get its accuracy.
  
## Installation

Clone the repository in your catkin workspace. Considering the usual ~/catkin_ws it would be as follows:

`cd ~/catkin_ws/src`

`git clone https://github.com/Carlita96/special_course.git`

### ROS Library

In order to use this, PCD files of the targets need to be recorded. The ROS code proposed here can be use or the viewer of the different sensors.
These are the SDKs of the sensors used:
  * Realsense L515: ![Realsense ROS SDK](https://github.com/IntelRealSense/realsense-ros)
  * Realsense D435i: ![Realsense ROS SDK](https://github.com/IntelRealSense/realsense-ros)
  * ZED2: ![ZED2 ROS SDK](https://github.com/stereolabs/zed-ros-wrapper)
  * Livox Mid-40: ![Livos ROS SDK](https://github.com/Livox-SDK/livox_ros_driver)
  * Mynteye D1000: ![Mynteye-D ROS SDK](https://github.com/slightech/MYNT-EYE-ROS-Wrapper)

### Target processing tools

In order to use the targetProcessTool or bitTargetProcessTool, build each of the libraries as follows from the special_course:

`cd ~/catkin_ws/special_course`

`cd tools/targetProcessTool/`

`mkdir build`

`cd build`

`cmake ..`

`make -j4`

Start from the beginning and change `cd tools/targetProcessTool/` for `cd tools/bitTargetProcessTool/` for building that tool.
 
## How to use

### ROS Library

Catkin make the directory:

`cd ~/catkin_ws`

`catkin_make`

Run the roslaunch of the desired sensor to record data. For example:

`roslaunch special_course recordDataLivox.launch`

Different parameters can be changed in the roslaunch files under /launch. For example, turn up or down the visualization, change the time integration of the point cloud, etc.

### PLY to PCD

In order to use this tool, change directory to its position and run it with python or python3. Remember to send also the directory which PLY files will be converted to PCDs.


`cd ~/catkin_ws/special_course/tools`

`python3 plyToPcd.py ~/Documents/recordedPly`

### Target processing tools

In order to use the targetProcessTool, go to the executable built before:

`cd ~/catkin_ws/special_course/tools/targetProcessTool/build`

and run it with the following information:
  1. Directory of PCD file.
  1. X of center right target.
  1. Y of center right target.
  1. Z of center right target.
  1. Size of cropbox for the target.
  1. Boolean to set if depth is Z axis (1) or X axis (0). 1 is default value.
,for example:

`./targetProcessTool ~/Documents/pcd.pcd 0 0 1.5 0.1 0`

In order to use the bigTargetProcessTool, go to the executable built before:

`cd ~/catkin_ws/special_course/tools/bigTargetProcessTool/build`

and run it with the following information:
  1. Directory of PCD file.
  1. X of center right target.
  1. Y of center right target.
  1. Z of center right target.
  1. Size of cropbox for the target.
,for example:

`./bigTargetProcessTool ~/Documents/pcd.pcd 0 0 1.5 0.2`

The information obtained from this tool will be printed in the terminal.
