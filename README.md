# Master's thesis: Detection of the navigable road limits by analysis of the accumulated point cloud density
Files and code included in the masters thesis 

# Installation

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
