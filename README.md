Aubo_Robot
===============================================================================================

This repository provides ROS support for the aubo robots.  
This repo holds source code for ROS versions Indigo.

__Operating System Install__  

Operating system version is not less than Ubuntu linux 14.04, both supports 32bit and 64bit system.
Ubuntu Linux download:http://www.ubuntu.com/download/

__Peak_Can drive Install__  

Version peak-linux-driver-8.1,download: http://www.peak-system.com/fileadmin/media/linux/


__Installation from Source__  

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/aubo_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile.

If there is  a linking dependency in aubo_driver with libev library. To solve it it is necessary to install it: 
```sudo apt-get install libev-dev```


__Usage with control real robot use Peakcan Tool___  

1.Make sure have installed Peakcan driver, connect peackcan to aubo robot i5, then run command,optional parameter(-S1,-S2,-S3) can control the joint move speed.

```rosrun aubo_control joint_control_pcan -S1```
  
   Note:default joint move speed is S1. 

2.A simple control panel in Rviz,run command line like this:

```roslaunch aubo_description aubo_i5_rviz.launch```
   
   Choose PCAN bus interface, and select "Continuous move" mode.
 
   Then we can control 6 joints with press button "+" and "-".



__Usage with control real robot use TCP/IP Server__  


   Firstly,check the Robot Controller's IP address,for example 192.168.1.34,then ping 192.168.1.34,make sure is connected. run command line like this:

```roslaunch aubo_driver aubo_i5_bringup.launch robot_ip:=192.168.1.34```
   
   there is a sim A simple control panel in Rviz

   Choose TCP/IP bus interface, and select "Move to Goal with AUBO Plan API "mode.
  
   Then we can adjust 6 joints position with press button "+" and "-",and also you can choose classic position. Then, push button "sendGoal".


__MoveIt! with a simulated robot in Gazebo__ 

Again, you can use MoveIt! to control the simulated robot.  
1.To bring up the simulated robot in Gazebo, run:

```roslaunch aubo_gazebo aubo_i5.launch```

2.For setting up the MoveIt! nodes to allow motion planning run:

  make sure the follow context at aubo_i5_moveit_planning_execution.launch file: 
  "<remap if="$(arg sim)" from="/follow_joint_trajectory" to="/arm_controller/follow_joint_trajectory"/>",

```roslaunch aubo_i5_moveit_config aubo_i5_moveit_planning_execution.launch sim:=true```

3.For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch aubo_i5_moveit_config moveit_rviz.launch config:=true```



__MoveIt! with a real robot use AUBO-i5__  

There is a trajectory demo for this part use Peakcan Tool,also can use TCP/IP bus interface.
1.Make sure have installed Peakcan driver, connect peackcan to aubo robot i5, then run command,optional parameter(-S1,-S2,-S3) can control the joint move speed.

```rosrun aubo_control joint_control_pcan -S1```

   Note:default joint move speed is S1. 

2.For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch aubo_i5_moveit_config demo.launch```

3.Start up a trajectory generator,which receive goal and make trajectory,run:

```rosrun aubo_trajectory trajectory_gen```

4.Start up trajectory goal,which subscribe trajectory points and publish to joint_control_pcan:

```rosrun aubo_trajectory trajectory_goal```

5.A control panel with real robot,run command:

```roslaunch aubo_description aubo_i5_rviz.launch```

   there is a sim A simple control panel in Rviz

   Choose PCAN bus interface, and select "Move to Goal with ROS Plan(moveit...)"mode.
  
   Then we can adjust 6 joints position with press button "+" and "-",and also you can choose classic position. Then, push button "sendGoal".


__RealTime Interface with a real robot depend on aubo_new_driver__  

  This new function use FollowJointTrajectoryAction server to control the AUBO-I5.
   
  ```roslaunch aubo_new_driver aubo_i5_bringup.launch robot_ip:=xxx.xxx.xxx.xxx```
	
	or

  ```roslaunch aubo_new_driver aubo_i5_ros_control.launch robot_ip:=xxx.xxx.xxx.xxx```
  
  there are two example in aubo_new_driver/test


__MoveIt,use ros control with a real robot depend on aubo_new_driver__  

1.Start up aubo new driver

```roslaunch aubo_new_driver aubo_i5_ros_control.launch robot_ip:=xxx.xxx.xxx.xxx```

2.For setting up the MoveIt! nodes to allow motion planning run:

  make sure the follow context at aubo_i5_moveit_planning_execution.launch file: 
  "<remap if="$(arg sim)" from="/follow_joint_trajectory" to="/pos_based_pos_traj_controller/follow_joint_trajectory"/>",

```roslaunch aubo_i5_moveit_config aubo_i5_moveit_planning_execution.launch sim:=true```

3.For starting up Rviz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch aubo_i5_moveit_config moveit_rviz.launch config:=true```

4.Drag the end effector of simulated Robot,then plan and excute.


__For more information,please contact the author__  
















