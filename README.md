# ros_pf
This package is used to applicate the particlefilter in combination with a RPLidar A2 for mobile robot localization in ROS.

## Installation & Packages
This package heavily relies on several packages to be installed and works on ROS Noetic.  
To install ROS Noetic one can follow the guide on the [official ROS page](http://wiki.ros.org/noetic/Installation/Ubuntu).

After successfully installing ROS Noetic these packages have to be installed:  
```$ sudo apt install ros-noetic-map-server```  
```$ sudo apt install ros-noetic-gmapping```  
```$ sudo apt install ros-noetic-amcl```  
```$ sudo apt install ros-laser-scan-matcher```  

To use the RPLidar the following package must be cloned into the src directory of the workspace:  
```$ git clone https://github.com/robopeak/rplidar_ros```

Then proceed to clone this package:  
```$ git clone https://github.com/Cion318/ros_pf```

As an alternative to the previously installed gmapping package one can also use the hector_slam package from TU Darmstadt:  
```$ git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam```

After all packages have been installed/cloned build them using ```catkin_make``` from the root of your workspace.

## Setup of RPLidar
Check the serial port with:  
```$ ls -l /dev |grep ttyUSB```  
Then add the write authority to the USB device (in my case ttyUSB0):  
```$ sudo chmod 666 /dev/ttyUSB0```

## Setup of Hector_Slam (only if used instead of gmapping):
Inside of ```<workspace>/src/hector_slam/hector_mapping/launch/mapping_default.launch``` change:  
* line 54 to ```<node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="0 0 0 0 0 0 base_link laser_link 100"/>```
* line  5 to ```<arg name="base_frame" default="base_link"/>```
* line  6 to ```<arg name="odom_frame" default="base_link"/>```

Inside of ```<workspace>/src/hector_slam/hector_slam_launch/launch/tutorial.launch``` change:
* line  7 to ```<param name="/use_sim_time" value="false"/>```
