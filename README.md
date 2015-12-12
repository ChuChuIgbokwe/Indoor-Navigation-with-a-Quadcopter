SLAM for Navigation of MAV's in Unknown Indoor Environments
===========================================================
This project is an extension of SLAM for Navigation of MAV's in Unknown Indoor Environments by Gian Danuser and Michael Eugester. 

###Table of Contents
[Objective](#Objective)  
[Setting up an internet connection  on the Pandaboard](#Setting up an internet connection  on the Pandaboard)  
[Setting up ROS](#Setting up ROS)  
[Project Overview](#Project Overview)  
[Building the pixhawk](#Building the pixhawk)  
[Getting Mavros and Mavlink](#Getting Mavros and Mavlink)  
[Main Scripts](#Main Scripts) 
[Setting up OpeNNI and OpenCV](#Setting up OpeNNI and OpenCV)
[Setting up visual odometry and sensor fusion](Setting up visual odometry and sensor fusion)
[Offboard Control with the Pixhawk](#Offboard Control with the Pixhawk)
[PandaBoard Issues](#PandaBoard Issues)
[Further Improvements](#Further Improvements)  
[Conclusions](#Conclusions)  

<a name="Objective"></a> 
###Objective
1. Install ROS on the PandaBoard
2.Establish communication with the Pixhawk
3.Fuse IMU data with pose estimates provided by the sensor fusion
4.Integrate low-level planning and the object avoidance unit
5.Implement the high-level navigation unit with a goal based trajectory


<a name="PandaBoard Issues"></a> 
###PandaBoard Issues
The PandaBoard overheats while compiling your ROS workspace. The board has two cores which means compiling any workspace defaults to using the -j2 flag. To successfully compile any package you have to specify that it runs with one flag like this
```
catkin_make -j1
```
Or else this happens
The down side to this is that a packages that requires dependencies from multiple packages like the multiple sensor framework package cannot compile
