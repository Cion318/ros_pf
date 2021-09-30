# Particlefilter application using amcl in ROS
This package is primarily used to apply the particlefilter in combination with a lidar (RPLidar in this case) for mobile (robot) localization in ROS.  
To make it as simple as possible there are multiple launch files to:
* map the environment using gmapping
* map the environment using hector_slam
* do a live localization when moving the lidar around(local and global)
* playing back recorded bagfiles created with the localization launch files


## Installation & Packages
This package heavily relies on several packages to be installed and works on ROS Noetic.  
To install ROS Noetic one can follow the guide on the [official ROS page](http://wiki.ros.org/noetic/Installation/Ubuntu).

After successfully installing ROS Noetic these packages have to be installed:  
* ```$ sudo apt install ros-noetic-map-server```  
* ```$ sudo apt install ros-noetic-gmapping```  
* ```$ sudo apt install ros-noetic-amcl```  
* ```$ sudo apt install ros-laser-scan-matcher```  

To use the RPLidar the following package must be cloned into the src directory of the workspace:  
* ```$ git clone https://github.com/robopeak/rplidar_ros```

Then proceed to clone this package:  
* ```$ git clone https://github.com/Cion318/ros_pf```

As an alternative to the previously installed gmapping package one can also use the hector_slam package from TU Darmstadt:  
* ```$ git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam```

After all packages have been installed/cloned build them using ```catkin_make``` from the root of your workspace.

## Setup of RPLidar
Check the serial port with:  
* ```$ ls -l /dev |grep ttyUSB```  
Then add the write authority to the USB device (in my case ttyUSB0):  
* ```$ sudo chmod 666 /dev/ttyUSB0```

## Setup of Hector_Slam (only if used instead of gmapping):
Inside of ```<workspace>/src/hector_slam/hector_mapping/launch/mapping_default.launch``` change:  
* Line  5 to ```<arg name="base_frame" default="base_link"/>```  
* Line  6 to ```<arg name="odom_frame" default="base_link"/>```  
* Line 54 to ```<node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="0 0 0 0 0 0 base_link laser_link 100"/>```  

Inside of ```<workspace>/src/hector_slam/hector_slam_launch/launch/tutorial.launch``` change:  
* Line  7 to ```<param name="/use_sim_time" value="false"/>```  

## Creating and Saving Maps
Like mentioned before one can use gmapping or hector_slam to map out the environment. To use either of them run one of these commands:
* ```$ roslaunch ros_pf gmap.launch```  
* ```$ roslaunch ros_pf hmap.launch```  

To save the generated map ```cd``` into the directory you want the map to be saved in and run the following command from a new terminal:
* ```$ rosrun map_server map_saver -f <map_name>```

## 