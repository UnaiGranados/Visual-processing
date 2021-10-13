# Vision based robot control in ROS

This ROS metapackage contains camera calibration tools and specific robot setup configuration files.

## 1. Dependencies

Tecnalia repositories:

* `algebra_libraries`
* `manipulation`


Additional dependencies:

* `fanuc`
* `industrial_robot_simulator`
* `robot_state_publisher`
* `rviz`
* `realsense-ros`

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
<ws_folder>/src $ git clone git@git.code.tecnalia.com:tecnalia_robotics/internships/unai-granados/fanuc_3d_cam.git
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

This is a set of packages to perform laser scanner calibration and validation. Only FANUC CR-7iA/L robot and Quelltech Q4 laser scanner have been tested so far. However, it is intended to extend these tools to some additional robots and laser scanners. The current implementation includes the following three main packages:

* `flexbotics_cr7ial_support`: configuration, scene URDF and launch files for the FANUC CR-7iA/L robot.
* `flexbotics_cr7ial_moveit_config`: *MoveIt! Setup Assistant* generated package.
* `laser_calibration`: this is where the nodes that perform the actual work are implemented.

There are another two packcages that are not directly related to calibration but contain nodes that might be useful for someone making use of the same elements (FANUC CR-7iA/L robot and Quelltech Q4 laser scanner):

* `laser_calibration_sandbox`: some dummy tests with the laser and `the manipulator_commander` package.
* `laser_scan`: a simple node that exposes two services `/laser_scan_node/start_capturing` and `/laser_scan_node/stop_capturing` to capture laser readings and robot poses. The node starts capturing data when the `start_capturing` service is called and stops capturing and publishes a point cloud of the captured data on the `/laser_scan_node/pointcloud` topic when the `stop_capturing` service is called.

## 4. Usage

### 4.1. Robot setup test scenario

The scene with the FANUC CR-7iA/L robot can be launched as follows:

```shell
$ roslaunch flexbotics_cr7ial_support set_up_cr7ial_scene.launch
```

By default, the scene will be launched with a simulated robot making use of the `industrial_robot_simulator` package. To launch the scene with the real robot set the `use_sim` argument to `false`:

```shell
$ roslaunch flexbotics_cr7ial_support set_up_cr7ial_scene.launch use_sim:=false
```


```shell
$ rosrun laser_calibration_sandbox calib_test_manipulator_node
```

For instructions about how to use the real robot see the [tecnalia_robotics/fanuc/Documentation](https://git.code.tecnalia.com/tecnalia_robotics/fanuc/documentation) repository.

### 4.2. Camera calibrator 

There is a dummy program that has been implemented in a node to test the Quelltech Q4 laser scanner. First, launch the driver with the following launchfile:

```shell
$ roslaunch laser_calibration calib_quelltech_laser.launch
```

By default, the driver is launched is simulation mode. To use the real device add the `sim_laser:=false` argument to the previous command. Then, the dummy node can be launched:

```shell
$ rosrun laser_calibration_sandbox calib_test_quelltech_node
```

This node activates the laser and performs readings during 5 seconds. Laser readings correspond to profiles where data is stored in the form of points in the X-Z plane. At the end of the program a PCL visualizer is launched where all the captured profiles are displayed stacked along the Y-axis.

### 4.3. Visual servoing

The approach implemented here is based on making a series of captures of a plane with a laser scanner from different robot flange positions. In this scenario, the laser scanner will capture straight lines. If the lines are expressed in the coordinate system of the robot base and the calibration is correct, these lines should converge onto a plane.

This package implements the data acquisition part of the algorithm. Robot movements are performed with the laser scanner facing a plane. The robot will loop over different poses and at each location will wait for 1 second before recording the robot pose and laser reading. All the data is then saved into a CSV file that can be fed to the Python program implemented in the [laser_calibration](https://git.code.tecnalia.com/tecnalia_robotics/avanwinglet/laser_calibration) repository where the actual calibration is performed.

To launch the data acquisition execute the following command:

```shell
$ roslaunch laser_calibration calib_acquisition.launch simulate_devices:=false
```

The robot will loop over 81 different poses assuming that the robot is mounted on top of a table as in the Tecnalia's Flexible Robotics lab. These position and rotations are set by default by the `calib_validation_node`. Check the input arguments to this node to change this behaviour.

### 4.4.  

The idea behind the calibration validation is to scan an element with a known geometry and check the accuracy of the reconstructed pointcloud from the laser readings. The program that allows capturing the data is implemented in the `laser_calibration/src/calib_validation_node.cpp` node. Data is generated by performing a linear movement with the robot and storing robot poses and laser readings in separate CSV files.

When operating with the real robot, the robot has to be positioned with the teach pendant over the element that we want to scan and the program will perform a linear movement of 10 cm along the X-axis of the base of the robot while the laser is active. The program can be launched with the following command:

```shell
$ roslaunch laser_calibration calib_validation.launch simulate_devices:=false output_folder:=<folder>
```

This launchfile has to be executed on its own without having to launch the previously described launchfiles for robot or laser configuration. Note that the output folder where the two CSV files will be stored has to be specified. In this example the `simulated_devices:=false` argument is also passed to make use of the real robot and laser instead of running simulated devices.
