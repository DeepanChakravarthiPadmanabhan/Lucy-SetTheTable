# Software Development Project 2019

## Title: Set the table

## Objective:
The objective of this project is to assist a domestic service robot, Lucy to set the table with the object provided by the user.

## Project overview:
The scope of the project is to provide the pose for each object to be placed on the table given the table and the item specifications as illustrated in the figure 1.

![projectoverview_img](https://user-images.githubusercontent.com/43172178/60401336-0b58d100-9b80-11e9-9d3a-ecfae472c0c5.png)

<p align="center">
  Figure 1: Project overview
</p>

## Component diagram of the project:
The entire project is divided into the following components as shown in figure 2.

![Setthetable](https://user-images.githubusercontent.com/43172178/60401302-7eae1300-9b7f-11e9-9b24-353a21b632ad.png)

<p align="center">
  Figure 2: Component diagram of the project
</p>

## Installations:
The major tools required are ROS Kinetic and Point Cloud Library.
The resources to follow for installation are ![ROS Kinetic](http://wiki.ros.org/kinetic/Installation), ![MoveIt](https://moveit.ros.org/) and ![PCL](http://www.pointclouds.org/downloads/).

## Procedure to compile and run:

### Compile
* Copy 'set_the_table' package inside your catkin worksapce src folder
* Copy the contents inside 'Table_setup_visualization' package inside your catkin worksapce src folder
* Copy the contents inside 'Input_mockup' package inside your catkin worksapce src folder

In your catkin worspace run the following commands to build the workspace,
```
catkin_make -j 1

```

### Run

* Run the roscore
* Launch the mockup node using the following command line in terminal, 
```
roslaunch convex_hull_inputnode convex_hull_inputnode.launch
```
* Run the item specification service, 
```
rosrun set_the_table table_configuration_node 
```
* Run the table setup node, 
```
 rosrun set_the_table table_setup_node
```
* Trigger the mockup to publish table pcl,
```
 rostopic pub /event_in std_msgs/String "data: 'true'"
```
* Run rviz for visualization,
```
 rosrun rviz rviz
```
* Set the number of item collections to place on the table,
```
 rostopic pub /table_setup_node/setup_count std_msgs/Int16 "data: 6"
```
* Trigger the setup with your custom algorithm,
```
 rostopic pub /table_setup_node/set_trigger std_msgs/String "data: 'set_table'" 
```

## Visualization

In the rviz tool you are require to setup the following,

* Set fixed frame as base
* Add marker with Marker topic: /table_visualizer
* Add a simple TF display type

## Team Members:

[Deepan Chakravarthi Padmanabhan](https://github.com/DeepanChakravarthiPadmanabhan)

[Muhammed Umer Ahmad Khan](https://github.com/umerkhan-mas)

[Sathiya Ramesh](https://github.com/Shisthruna28)

