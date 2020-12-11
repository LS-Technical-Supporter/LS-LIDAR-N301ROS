## Describe
	**This version only verifies the Single line n301 radar with dual ports**
	**The driver is developed with ros1, and supports running under Ubuntu 14.04, Ubuntu 16.04, and Ubuntu 18.04**

## Set up the workspace
```
mkdir -p ~/lidar_ws/src
cd ~/lidar_ws/src
tar –xvf lidar_n301_V2.01.tar
```

## Compile and package
```
cd ~/lidar_ws
catkin_make
```	

## Run
```
source ~/lidar_ws /devel/setup.bash
roslaunch lidar_n301_decoder lidar_n301.launch
```



## lidar_n301.launch Configuration file description: 
~~~xml
	<arg name="device_ip" default="192.168.1.206" />	//Set to the corresponding IP of radar
	<arg name="device_port" default="2366" />	//The port corresponding to the data packet
	<param name="frame_id" value="laser_link"/>	//Set the name of the fixed frame in rviz
	<param name="add_multicast" value="false"/>	//false means Turn off multicast
	<param name="group_ip" value="224.1.1.2"/>	//Multicast IP settings
	<param name="min_range" value="0.15"/>	//Points less than 0.15 meters are not displayed
	<param name="max_range" value="150.0"/>	//Points greater than 150 meters are not displayed
	<param name="angle_disable_min" value="100"/>	//Points from 100 to 300 degrees are not displayed
	<param name="angle_disable_max" value="300"/>
	<param name="use_gps_ts" default="false" />	//False means to use the local time of the computer, and true means to use the GPS time
	<param name="gps_correct" value="true"/>	//False means that GPS time correction is not turned on, and true means that GPS time correction is turned on
    <param name="publish_point_cloud" value="true"/>	//False means that pointcloud point cloud is not displayed, and true means that pointcloud point cloud is displayed
    <param name="filter_scan_point" value="true"/>	//True means to filter a circle of redundant points, false means not to filter
~~~







