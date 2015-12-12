SLAM for Navigation of MAV's in Unknown Indoor Environments
===========================================================
This project is an extension of *SLAM for Navigation of MAV's in Unknown Indoor Environments* by Gian Danuser and Michael Eugester. 

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

<a name="Setting up an internet connection  on the Pandaboard"></a> 
###Setting up an internet connection  on the Pandaboard
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
sudo apt-get install wpa-supplicant
sudo spt-get install isc-dhcp-client
sudo apt-get interfavahi-autoipd
sudo apt-get install avahi-autoipd

sudo iwlist wlan0 essid []
sudo iwlist eth0 essid []
sudo apt-get install dhclient
apt-cache search dhclient

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

<a name="Further Improvements"></a>
###Further Improvements

+ Work on the project is still ongoing. The biggest impediment to testing out the code is the connection from the pixhawk to the pandaboard. The FTDI to USB cord from the pixhawk shows up as /tty/dev/ACM0 on the Pandaboard. Changing it to /tty/dev/USB0 in the px4.launch file allows the launch file to run. However a Heartbeat isn't gotten from the pixhawk, neither is any other message. 
  Using a USB to USB connection from the Pandaboard to the Pixhawk works, all the messages from the pixhawk are received by the board. This isn't a solution as USB-USB connections on the pixhawk time-out after 30 seconds of not receiving any signal. This is highly undesirable during flight. I haven't been able to resolve this issue yet, I'm confident I will during the coming weeks
+ Using a more powerful or better supported board to bypass the aforementioned issues and to implement real time SLAM and sensor fusion on the board.

<a name="Conclusions"></a>
###Conclusions 
