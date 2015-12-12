SLAM for Navigation of MAV's in Unknown Indoor Environments
===========================================================
This project is an extension of *SLAM for Navigation of MAV's in Unknown Indoor Environments* by Gian Danuser and Michael Eugester. 

###Table of Contents
+ [Objectives](#Objectives)  
+ [Project Overview](#Project Overview) 
+ [Setting up an Internet Connection  on the Pandaboard](#Setting up an Internet Connection  on the Pandaboard)  
+ [Setting up ROS](#Setting up ROS)  
+ [Building the Pixhawk](#Building the Pixhawk)  
+ [Getting Mavros and Mavlink](#Getting Mavros and Mavlink)  
+ [Setting up visual odometry and sensor fusion](#Setting up visual odometry and sensor fusion)
+ [Offboard Control](#Offboard Control)
+ [What's been Done,Issues and Tips](#What's been Done,Issues and Tips)
+ [Further Improvements](#Further Improvements)  
+ [Conclusions](#Conclusions)  
+ [References](#References)

<a name="Objectives"></a> 
###Objectives
1. Install ROS on the PandaBoard
2. Establish communication with the Pixhawk
3. Fuse IMU data with pose estimates provided by the sensor fusion
4. Integrate low-level planning and the object avoidance unit
5. Implement the high-level navigation unit with a goal based trajectory

<a name="Project Overview"></a> 
###Project Overview
The long term  goals of this project is to implement real-time SLAM, autonomous flight,obstacle avoidance and object detection on a quadcopter equipped with a PandaBoard and an ASUS XTION PRO. 
The project builds off previous work that got SLAM working on a laptop, and radio controlled flight of the quadcopter. Fusing these two parts is the major objective of this project.
The SLAM code used in this project was written from the ground up, including drivers for the ASUS Xtion PRO. The code works, however it is not optimised for the PandaBoard. The solution to this was to use ROS (Robot Operating System) as it has pre-existing packages for SLAM and algorithms can easily be written for autonomy, obstacle avoidance and object Detection.

<a name="Setting up an Internet Connection  on the Pandaboard"></a> 
###Setting up an Internet Connection  on the Pandaboard
This is the first step as it precludes all the others. The first thing is to check if your access to the internet is being blocked. You check this with
```
rfkill list
```
In this case the soft block was on which means the connection was being blocked by software. This can be solved by
```
sudo rfkill unblock all
```
Scan for available networks with
```
wicd-curses
```
or 
```
sudo iwlist wlan0 scan
```
```
sudo apt-get install wpa-supplicant
sudo spt-get install isc-dhcp-client
sudo apt-get interfavahi-autoipd
sudo apt-get install avahi-autoipd

sudo iwlist wlan0 essid [connection you want to connect to]
sudo iwlist eth0 essid [connection you want to connect to]
sudo apt-get install dhclient
apt-cache search dhclient
```
once found configure your /etc/network/interfaces
```
sudo nano /etc/network/interfaces
auto lo
iface eth0 inet dhcp
iface wlan0 inet dhcp
iface lo inet loopback
auto wlan0
iface wlan0 inet dhcp
wpa-driver wext
wpa-ssid <--NAME OF AP-->
wpa-ap-scan 1
wpa-proto RSN
wpa-pairwise CCMP
wpa-group CCMP
wpa-key-mgmt WPA-PSK
wpa-psk <--INSERT KEY XXXXXXXXXXXXXXXXX-->
```

<a name="Setting up ROS"></a> 
###Setting up ROS

Run the following commands
```
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-jade-ros-base 
sudo apt-get install ros-jade-usb-cam ros-jade-mavlink ros-jade-mavros ros-jade-cv-bridge ros-jade-image-proc
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/jade/setup.bash
echo "source /opt/ros/indigo/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```
To create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make #Even though the workspace is empty, it "compiles" which should tell you that your new setup is fine.
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc # auto sources workspace in new bash sessions
source ~/.bashrc
```

<a name="Building the Pixhawk"></a>
###Building the Pixhawk
[The steps for building the pixhawk can be found here!] (http://dev.px4.io/starting-installing-linux.html)

<a name="Getting Mavros and Mavlink"></a> 
###Getting Mavros and Mavlink
MAVLink is a very lightweight, header-only message marshalling library for micro air vehicles according to the official documentation. It is a communication protocol used for communication between an autonomous vehicle - it's auto pilot or flight controller exactly- and fellow autonomous vehicles, it's ground station or its onboard computer. 
Basically mavlink is a stream of bytes sent via telemetry or USB from the auto pilot to the ground control station.
The two most popular firmwares that use mavlink are the APM and PX4. In this project the Pixhawk which has a PX4 build was used.
Each MavLink packet has a length of 17 bytes 
+ 6 bytes header
+ message header, always 0xFE
+ message length (9)
+ sequence number -- rolls around from 255 to 0 (0x4e, previous was 0x4d)
+ System ID - what system is sending this message (1)
+ Component ID- what component of the system is sending the message (1)
+ Message ID 
Variable Sized Payload (specified in octet 1, range 0..255)
** Payload (the actual data we are interested in)
Checksum: For error detection.

The most important message is MAVLINK_MSG_ID_HEARTBEAT. The GCS keeps sending a message to APM/PX4 to
find out whether it is connected to it (every 1 second). This is to make sure the MP is in sync with APM when you update some parameters. If a number of heartbeats are missed, a failsafe (can be) is triggered and copter lands, continues the mission or Returns to launch (also called, RTL).

Mavros is a MAVLink extendable communication node for ROS with proxy for Ground Control Station. It's Mavlink ported to ROS.It allows you to send commands to the quadcopter via ROS communication protocols. This is the basis of autonomous and teleop flight.

<a name="Setting up visual odometry and sensor fusion"></a>
###Setting up visual odometry and sensor fusion
Vision pose estimation is implemented in ROS using two packages
+ Semi-Direct Visual Odometry (SVO)
+ Multiple Sensor Fusion Framework (MSF)

The semi-direct approach eliminates the need of costly feature extraction and robust matching techniques for motion estimation. The algorithm operates directly on pixel intensities, which results in subpixel precision at high frame-rates. A probabilistic mapping method that explicitly models outlier measurements is used to estimate 3D points, which results in fewer outliers and more reliable points. Precise and high frame-rate motion estimation brings increased robustness in scenes of little, repetitive, and high-frequency texture. The algorithm is applied to micro-aerial-vehicle state-estimation in GPS-denied environments and runs at 55 frames per second on the onboard embedded computer and at more than 300 frames per second on a consumer laptop. 

The MSF is based on an Extended Kalman Filter (EKF) for 6DOF pose estimation including intrinsic and extrinsic sensor calibration. It combines pose estimates from the Pixhawk via Mavros with the with the pose the SVO package gets from the camera to produce accurate pose estimates of the quadcopter. This allows the quadcopter to be used for applications such as position hold indoors or waypoint navigation based on vision.
![rqt_graph](https://cloud.githubusercontent.com/assets/4311090/11760816/0374e956-a070-11e5-9276-4d7b3892a822.png)

The SVO package requires two workspaces, one for the plain CMake projects Sophus and Fast and another workspace for the ROS-Catkin projects rpg_vikit and rpg_svo.
```
sudo apt-get install libeigen3-dev libeigen3-doc
mkdir ~/workspace #for the cmake projects
cd ~/workspace
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make
 
cd ~/workspace
git clone https://github.com/uzh-rpg/fast.git
cd fast
mkdir build
cd build
cmake ..
make
```
The following steps show how to get the remaining packages installed anb build the catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/uzh-rpg/rpg_vikit.git
git clone https://github.com/uzh-rpg/rpg_svo.git
git clone https://github.com/ethz-asl/asctec_mav_framework
git clone https://github.com/ethz-asl/catkin_simple
git clone https://github.com/ethz-asl/gflags_catkin.git
git clone https://github.com/ethz-asl/glog_catkin
https://github.com/ethz-asl/geodetic_utils.git
git clone https://github.com/ethz-asl/ethzasl_msf
```
go to line 200 in ethzasl_msf/msf_core/include/msf_core/msf_sensormanagerROS.h and change
```
if (pubPoseCrtl_.getNumSubscribers() || pubPose_.getNumSubscribers() || pubOdometry_.getNumSubscribers()) {
```
to 
```
if (true){
```
remove subscriber detection by the MSF Package.
check dependencies and compile
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
catkin_make -j2
```

<a name="Offboard Control"></a>
###Offboard Control
![Offboard flowchart](https://cloud.githubusercontent.com/assets/4311090/11760392/895c72bc-a05e-11e5-9e2c-baacbeae79d3.png)
Offboard control is controlling a PX4 based MAV using off-board control software. Ths setup requires a Onboard computer + ROS + WiFi link. This setup will ultimately give the most flexibility. Using ROS simplifies offboard control  as well as providing access to a wide range of motion planning libraries and algorithms currently available in ROS. The WiFi link to the ground computer will also provide a high-bandwidth connection for high stream rates of commands
The onboard computer is connected to the pixhawk via UART. This project used a UART to USB cable with a DF13 connector plugged into TELEM2 of the Pixhawk
![FTDI to USB cable in TELEM2 of pixhawk](https://cloud.githubusercontent.com/assets/4311090/11760441/58d2ce68-a061-11e5-9c74-615b0440b057.jpg)
Connect the pixhawk to your the Pandaboard and run
```
roslaunch mavros px4.launch
```
If everything has been setup right you should see
```
[ INFO] [1423604938.457425540]: CON: Got HEARTBEAT, connected.
```
After that you need to create a px4_blacklist.yaml
```
cd /opt/ros/jade/share/mavros
touch px4_blacklist.yaml
````
copy 
```
plugin_blacklist:
- '3dr_radio'
- 'setpoint_position'
- 'setpoint_attitude'
#- 'setpoint_accel'
- 'setpoint_velocity'
- 'actuator_control'
```
into it.
Open another tab and run
```
rosrun mavros mavsys mode -c OFFBOARD
```
To enable offboard control on FMU (offboard from FMU perspective) one needs to send following commands:
+ Use set_mode service, from Mavros plugin sys_status, to set mode to “OFFBOARD”.
+ start streaming offobard setpoints.
+ send MAV_CMD_NAV_GUIDED_ENABLE Mavros command until request is approved.
After this you should open another tab and rosrun or roslaunch your script.


<a name="What's been Done,Issues and Tips"></a> 
###What's been Done and Issues
+ ROS Jade was successfully installed. ROS Indigo would be preferred as it is more stable. I tried both and settled on ROS Jade as it was the distro I was able to install the SVO package on.
+ Communication between the Pixhawk and the onboard computer has been achieved
+ Openni2 drivers work for the ASUS Xtion Pro
+ I haven't attempted onboard control or arming the motors for reasons stated below.
+ I wasn't able to compile The MSF on any ROS distro for the PandaBoard
+ The SVO package works
+ + gcc and cmake are tricky. I had to reinstall them multiple times.
+ I had to uninstall Gian and Michaels OpenCV as it prevented packages from compiling.
+ The full set of commands for connecting to the internet can be found on the PandaBoard's home directory in the file pandaboard_history.txt
+ Setting up the Pandaboard to connect to the internet via Northwestern's IP is tricky. An easir solution is to use an ethernet cord to share the internet connection from your laptop.
+ The PandaBoard overheats while compiling your ROS workspace. The board has two cores which means compiling any workspace defaults to using the -j2 flag. To successfully compile any package you have to specify that it runs with one flag like this
```
catkin_make -j1
```
![j1 compilation success](https://cloud.githubusercontent.com/assets/4311090/11759525/3d1d12fc-a041-11e5-9fa1-e79913ad2098.png)
Or else this happens, Overheating!
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
+ Do not try to install qgrouncontrol from source. It has too many dependency problems. Simply use
```
sudo apt-get install qgroundcontrol
```


<a name="Further Improvements"></a>
###Further Improvements

+ Work on the project is still ongoing. The biggest impediment to testing out the code is the connection from the pixhawk to the pandaboard. The FTDI to USB cord from the pixhawk shows up as /tty/dev/ACM0 on the Pandaboard. Changing it to /tty/dev/USB0 in the px4.launch file allows the launch file to run. However a Heartbeat isn't gotten from the pixhawk, neither is any other message. 
  Using a USB to USB connection from the Pandaboard to the Pixhawk works, all the messages from the pixhawk are received by the board. This isn't a solution as USB-USB connections on the pixhawk time-out after 30 seconds of not receiving any signal. This is highly undesirable during flight. I haven't been able to resolve this issue yet, I'm confident I will during the coming weeks
+ Using a more powerful or better supported board to bypass the aforementioned issues and to implement real time SLAM and sensor fusion on the board. Everyboard referenced in the ROS and Pixhawk wikis is at the least quad core boards.The PandaBoard is a Dual-Core board
+ Use the SVO package for visual odometry. It's works using usb cameras. Using the ASUS Xtion Pro will require topic remapping and writing nodelets for the ASUS.

<a name="Conclusions"></a>
###Conclusions 
Achieving the goals of this project within the next couple of weeks is totally feasible.The Pandaboard has been a blessing and a curse. The blessing being that I've gained a very good understanding of Linux ARM systems along with the debugging skills, tips and tricks. My frustration is I do no have much to show for all I've learnt however with the right hardware I can get this project up and runnig quickly.


<a name="References"></a>
###References
1. MavLink Tutorial for Absolute Dummies 
2. *SVO: Fast Semi-Direct Monocular Visual Odometry*  Christian Forster, Matia Pizzoli, Davide Scaramuzza
3. www.pixhawk.org
4. www.http://qgroundcontrol.org
5. www.diydrones.com

