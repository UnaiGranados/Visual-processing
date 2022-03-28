# 3D vision system for tag detection and dynamic control of robotic arm in ROS

This ROS metapackage contains specific robot setup configuration, tag detections  and robot control files.

## 1. Dependencies


Additional dependencies:

* `fanuc`
* `fanuc_gazebo`
* `industrial_robot_simulator`
* `robot_state_publisher`
* `rviz`
* `rviz_visual_tools`
* `realsense-ros`
* `realsense_gazebo_plugin`
* `moveit_calibration`
* `moveit_visual_tools`

## 2. Installation

### 2.1. Create catkin workspace

```shell
$ mkdir -p <ws_folder>/src
$ cd <ws_folder>
```

### 2.2. Initialize workspace

```shell
<ws_folder> $ catkin init
```

### 2.3. Download source code

```shell
<ws_folder> $ cd src
<ws_folder>/src $ git clone https://github.com/UnaiGranados/Visual-processing.git
```

### 2.4. Synchronize depencies

```shell
<ws_folder>/src $ wstool init
<ws_folder>/src $ wstool merge <package_folder>/.rosinstall
```

### 2.5. Clone repositories defined in the `.rosinstall` file                                                                                                                                                                                                                              
```shell
<ws_folder>/src $ wstool update -t .
```

### 2.6. Install dependencies

Before running `rosdep install` make sure lists are updated with `sudo apt update` and `rosdep update`.

```shell
<ws_folder>/src $ rosdep install --from-paths . -i --rosdistro=<ros-distro> -y
```

### 2.7. Compile workspace

```shell
<ws_folder>/src $ catkin build
```

## 3. Structure

This is a set of packages to perform  visual servoing functionalities. The current implementation includes the following three main packages:

* `flexbotics_cr7ial_support`: configuration, scene URDF and launch files for the FANUC CR-7iA/L robot.
* `flexbotics_cr7ial_moveit_config`: *MoveIt! Setup Assistant* generated package.
* `visual_processing`: this is where the nodes and other packages that perform the actual work are implemented.


## 4. Usage

### 4.1. Robot setup test scenario

The scene with the FANUC CR-7iA/L robot can be launched in as follows:

```shell
$ roslaunch flexbotics_cr7ial_support set_up_cr7ial_scene.launch
```

By default, the scene will be launched with a simulated robot making use of the `industrial_robot_simulator` package. To launch the scene with the real robot set the `use_sim` argument to `false`:

```shell
$ roslaunch flexbotics_cr7ial_support set_up_cr7ial_scene.launch use_sim:=false
```

By default simulation will be visualized on Gazebo with a simulated RealSense D435 camera. To launch scene with the real camera set `use_gazebo` and `use_rs_gazebo` arguments to `false`:

```shell
$ roslaunch flexbotics_cr7ial_support set_up_cr7ial_scene.launch use_gazebo:=false use_rs_gazebo:=false
```

### 4.3. Tag detection

The approach implemented here is based on different python nodes to detect a marker/tag and compute the transform between camera and tag using OpenCV. To get this result, has been implemented two methods:

The firts one is using RGB image and tag model dimension to calculate the rotation and translation of the tag respect to the camera:

```shell
$ python tf_transform_rgb.py
```
The other one is using both RGB and depth image:

```shell
$ python tf_depth.py
```

### 4.4. Move robot 

The idea behind this is to move robot reading tags positions and tracking them. This node has been implemented using MoveIt. So `use_moveit` argument has to be set to `true` in launch file.

This node has been developed to approach camera to an offset position (50 cm in Z) from tag and can be launched with the following command:

```shell
$ python moveit_move_group.py 
```

