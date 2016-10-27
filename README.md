Aubo_Robot
===============================================================================================

This repository provides ROS support for the aubo robots.  
This repo holds source code for ROS versions Indigo.

__Operating System Install__  

Operating system version is not less than Ubuntu linux 14.04, both supports 32bit and 64bit system.
Ubuntu Linux download:http://www.ubuntu.com/download/

__Peak_Can drive Install__  

Version peak-linux-driver-8.1,download: http://www.peak-system.com/fileadmin/media/linux/

__Qt5 Widgets Install__  

Version:qt-opensource-linux-x64-5.6.1-1,download: https://www.qt.io/download-open-source/#section-2

__Installation from Source__  

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/aubo_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile.

If there is  a linking dependency in aubo_driver with libev library. To solve it it is necessary to install it: 
```sudo apt-get install libev-dev```

__Usage with rviz Simulation__  

Run command,then control joint position with state_publisher GUI.

```roslaunch aubo_description aubo_i5_rviz.launch```


__Usage with control real robot directly use Peakcan Tool__  

1.Make sure have installed Peakcan driver, connect peackcan to aubo robot i5, then run command,optional parameter(-S1,-S2,-S3) can control the joint move speed.

```rosrun aubo_control joint_control_pcan -S1```
  
   Note:default joint move speed is S1. 

2.A simple gui tool based Qt5,can moves the robot to predefined positions can be executed like this:

```rosrun aubo_control control_panel```
   
   Choose PCAN Direct Mode, then we can control 6 joints with press button "+" and "-".


3.Farther,if you want to see rviz synchronize with real robot,then run command:

```roslaunch aubo_description aubo_i5_rviz.launch```

```rosrun aubo_control real_state_rviz```


__Usage with control real robot use TCP/IP Server__  

1.Firstly,check the Robot Controller's IP address,for example 192.168.1.34,then ping 192.168.1.34,make sure is connected. run command line like this:

```roslaunch aubo_driver aubo_i5_bringup.launch robot_ip:=192.168.1.34```
  
   other,if you don't ned Rviz,you can run this:

```rosrun aubo_driver aubo_driver 192.168.1.34 8887```

2.A simple gui tool based Qt5,can control the robot to predefined positions can be executed like this:

```rosrun aubo_control control_panel```

   Choose TCP/IP Mode, we can adjust 6 joints position with press button "+" and "-",and also you can choose classic position. Then, push button "sendGoal".


__MoveIt! with a simulated robot in Gazebo__ 

Again, you can use MoveIt! to control the simulated robot.  
1.To bring up the simulated robot in Gazebo, run:

```roslaunch aubo_gazebo aubo_i5.launch```

2.For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch aubo_i5_moveit_config aubo_i5_moveit_planning_execution.launch sim:=true```

3.For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch aubo_i5_moveit_config moveit_rviz.launch config:=true```


There is another actionlib demo with a simulated robot in Gazebo.

1.To bring up the simulated robot in Gazebo, run:

```roslaunch aubo_gazebo aubo_i5.launch```

2.Use FollowJointTrajectoryAction to send motion goal,run:

```rosrun aubo_trajectory trajectory_client```

3.A simple gui tool based Qt5,can control the robot to predefined positions can be executed like this:

```rosrun aubo_control control_panel```

   Choose Moveit Plugin Mode, we can adjust 6 joint position with press button "+" and "-",and also you can choose classic position.Then, push button "sendGoal".



__MoveIt! with a real robot use Peakcan Tool__  

There is a trajectory demo for this part.  
1.Make sure have installed Peakcan driver, connect peackcan to aubo robot i5, then run command,optional parameter(-S1,-S2,-S3) can control the joint move speed.

```rosrun aubo_control joint_control_pcan -S1```

   Note:default joint move speed is S1. 

2.For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch aubo_i5_moveit_config demo.launch```

3.Start up a trajectory generator,which receive goal and make trajectory,run:

```rosrun aubo_trajectory trajectory_gen```

4.Start up trajectory goal,which subscribe trajectory points and publish to joint_control_pcan:

```rosrun aubo_trajectory trajectory_goal```

5.A simple gui tool based Qt5,can control the robot to predefined positions can be executed like this:

```rosrun aubo_control control_panel```

   Choose Moveit Plugin Mode, we can adjust 6 joint position with press button "+" and "-",and also you can choose classic position.Then, push button "sendGoal".

6.Farther,if you want to see rviz synchronize with real robot,then run command:

```roslaunch aubo_description aubo_i5_rviz.launch```

```rosrun aubo_control real_state_rviz```










