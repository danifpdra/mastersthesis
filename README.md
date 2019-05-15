# Master's thesis: Detection of the navigable road limits by analysis of the accumulated point cloud density
Files and code included in the masters thesis 

# Installation

This program was created for ROS melodic. For more informations go to http://wiki.ros.org/melodic. 

It is necessary to install the following packages: 
- novatel-gps-driver
- pcl-ros
- grid-map 
- (...)

To install a package, use:

```bash
sudo apt install ros-melodic-$PACKAGE_NAME$
```

Install OpenCV by running:

```bash
[compiler] sudo apt-get install build-essential
[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

To install PCL, please read http://www.pointclouds.org/downloads/linux.html. 

# Usage

Inside the catkin_ws, first compile the package driver_base:

```bash
catkin_make --pkg driver_base
```
Then, run:

```bash
catkin_make && roslaunch negative_obstacles negative_obstacles.launch
```

If it is necessary to run a rosbag (example for Volta_UA.bag):

```bash
rosbag play Volta_UA.bag --clock --topic /DadosInclin /ld_rms/scan0 /ld_rms/scan1 /ld_rms/scan2 /ld_rms/scan3 /imu /gps /fix /inspva /camera/image_color
```


# Credits
Created by Daniela Rato (danielarato@ua.pt)
