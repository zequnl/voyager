* [编译环境](#dependencies)
* [编译](#building)
* [运行](#running)
## <a name="dependencies"></a> 编译环境
* Ubuntu 16.04
* ROS Kinetic Kame 
* Catkin
* Gazebo 
* Hector ROS packages 
* Octomap ROS packages 
* Rviz 
* Teleop twist keyboard 
* Octovis 

## <a name="building"></a> 编译  
将该文件放在~/catkin_ws/src下
在~/catkin_ws/src中安装hector quadrotor包
```
$ cd ~/catkin_ws/src
$ wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/kinetic-devel/tutorials.rosinstall
$ cd ..
$ catkin_make
$ cd src
```
安装octomap
```
$ sudo apt-get install ros-kinetic-octomap
$ sudo apt-get install ros-kinetic-octomap-mapping 
$ rosdep install octomap_mapping 
$ rosmake octomap_mapping
```
安装octomap_rviz_plugins
```
$ git clone https://github.com/OctoMap/octomap_rviz_plugins.git
$ cd ..
$ catkin_make
```
安装ros-kinetic-octovi
```
$ sudo apt-get install ros-kinetic-octovis
```

安装ros-kinetic-teleop-twist-keyboard
```
$ sudo apt-get install ros-kinetic-teleop-twist-keyboard
```
编译整个工程
```
$ cd ..
$ catkin_make
```

## <a name="running"></a> 运行 
首先运行world.launch文件
```
$ roslaunch voyager world.launch
```
然后运行我们的机器人
```
$ roslaunch voyager voyager.launch
```
让机器人停止构建地图
```
$ rosservice call /explore false
```
