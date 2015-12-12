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
2. Establish communication with the Pixhawk
3. Fuse IMU data with pose estimates provided by the sensor fusion
4. Integrate low-level planning and the object avoidance unit
5. Implement the high-level navigation unit with a goal based trajectory




<a name="PandaBoard Issues"></a> 
###PandaBoard Issues
+ The PandaBoard overheats while compiling your ROS workspace. The board has two cores which means compiling any workspace defaults to using the -j2 flag. To successfully compile any package you have to specify that it runs with one flag like this
```
catkin_make -j1
```
![j1 compilation success](https://cloud.githubusercontent.com/assets/4311090/11759525/3d1d12fc-a041-11e5-9fa1-e79913ad2098.png)
Or else this happens
![-j2 compilation error](https://cloud.githubusercontent.com/assets/4311090/11759503/670c1a32-a040-11e5-8b32-0d605d9cb48c.png)
The down side to this is that a packages that requires dependencies from multiple packages like the multiple sensor framework package cannot compile
![large package failure](https://cloud.githubusercontent.com/assets/4311090/11759504/694fbb00-a040-11e5-80bb-65942e84acff.png)

+ Connecting the ASUS xtion PRO to the first USB port of the Pandaboard causes it reboot endlessly. I suspect the ASUS draws too much current from it.However it if connected to the second (lower) USB port it doesn't shut the system down.
+ I did most of my work on the PandaBoard via minicom. 
```
sudo minicom -D /dev/tty/USB0
```
  Minicom is a command line interface for UART to USB communication.The Pandaboard shuts down intermitently. Initially I     suspected this was due to the Xubuntu Desktop Environment I installed on the board. However I also realised that the       board shuts down randomly regardless of what platform you're using. Also since it's a dual core board compiling big        packages that take a long time to compile like PCL cannot be done overnight as the board will reset sometime during the    compilation.

+ The Pandaboard is lacking in online support. A lot of questions are unanswered and when they are they are years old or they are current answers for more popular development boards.
