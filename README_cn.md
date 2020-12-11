## 描述
	**本版本只验证了单线N301雷达双端口**
	**驱动用ros1进行开发，支持ubuntu14.04,ubuntu16.04,ubuntu18.04下运行**
## 建立工作空间
```
mkdir -p ~/lidar_ws/src
cd ~/lidar_ws/src
tar –xvf lidar_n301_V2.01.tar
```
## 编译打包
```
cd ~/lidar_ws
catkin_make
```	
	
## 运行: 
```
source ~/lidar_ws /devel/setup.bash
roslaunch lidar_n301_decoder lidar_n301.launch
```



## lidar_n301.launch配置文件说明: 
~~~xml
	<arg name="device_ip" default="192.168.1.206" />	//设置为雷达对应的IP
	<arg name="device_port" default="2366" />	//数据包对应的端口
	<param name="frame_id" value="laser_link"/>	//设置rviz中Fixed Frame的名称
	<param name="add_multicast" value="false"/>	//关闭组播
	<param name="group_ip" value="224.1.1.2"/>	//组播IP设置
	<param name="min_range" value="0.15"/>	//距离小于0.15米的点不显示
	<param name="max_range" value="150.0"/>	//距离大于150米的点不显示
	<param name="angle_disable_min" value="100"/>	//不显示从100度到300度范围的点
	<param name="angle_disable_max" value="300"/>
	<param name="use_gps_ts" default="false" />	//false表示使用电脑本地时间,true表示使用GPS时间
	<param name="gps_correct" value="true"/>	//false表示不开启GPS时间校正,true表示开启GPS时间校正
    	<param name="publish_point_cloud" value="true"/>	//false表示不显示point_cloud点云,true表示显示point_cloud点云
    	<param name="filter_scan_point" value="true"/>	//true表示过滤一圈多余的点，false表示不过滤
~~~







