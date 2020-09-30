# ELECTRIC POWERED 1/10 AUTONOMOUS CAR
This project covers the development of an autonomous electric car at a 1:10 scale. The car will use vision and LiDAR information to follow a track at high-speeds.

## Requirements
### Hardware
1. Electric Powered 1/10 RC Car
2. Hokuyo LiDAR UST-10LX
3. Jetson TX2
4. Orbitty Carrier
5. Arduino Nano Every

### Operating System
1. [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)

2. [JetPack 3.3](https://developer.nvidia.com/embedded/jetpack-3_3)

3. [Orbitty L4T R28.2X BSP](https://connecttech.com/ftp/Drivers/CTI-L4T-V121.tgz)

### Software
1. [Desktop-Full ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

2. [Python 2.7](https://www.python.org/download/releases/2.7/)
   - [pip](https://pypi.org/project/pip/)
   - [rospy](http://wiki.ros.org/rospy)
   - [NumPy](https://pypi.org/project/numpy/)
   
3. [Git](https://git-scm.com/download/linux)

4. [Arduino IDE](https://www.arduino.cc/download_handler.php)

## Installation on Host Computer
1. Install [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)

2. Install [JetPack 3.3](https://developer.nvidia.com/embedded/jetpack-3_3)

3. Install [Orbitty L4T R28.2X BSP](https://connecttech.com/ftp/Drivers/CTI-L4T-V121.tgz)

4. Install [Arduino IDE](https://www.arduino.cc/download_handler.php)

## Flashing the Jetson TX2
1. Connect your host computer to the Jetson TX2 via Micro-USB

2. Go through the full initial JetPack 3.3 installation

3. Copy the CTI-L4T-V121.tgz package into <installed_directory>/64_TX2/Linux_for_Tegra in the host machine

4. Extract the package
   - Open your terminal
   - Change to the Linux_for_Tegra directory
   - Type `tar -xzf CTI-L4T-V121.tgz`
   
5. Change into the CTI-L4T directory
   - Type `cd CTI-L4T`
   
6. Run the install script
   - Type `sudo ./install.sh`
   
7. Put the Jetson TX2 into recovery mode
   - Turn off the power
   - Turn on the power
   - Press the reset button
   - Hold the recovery button and press then reset button once
   - Keep holding the recovery button for 2 seconds

8. Change to the Linux_for_Tegra directory
   - Type `cd ..`

9. Flash the Jetson TX2
   - Type `./flash.sh orbitty mmcblk0p1`
   
10. Connect the Jetson TX2 system to the same network as your host machine and determine the local IP address of the Jetson TX2
   - Type `ifconfig` on the Jetson TX2
   
11. Re-launch the JetPack installer on the host computer
   - Type `cd ../..`
   - Type `./JetPack-L4T-3.3-linux-x64_b39.run`
   
12. Select the Jetson TX2

13. Install the default Jetson TX2 dependencies that have been overwritten by the BSP
   - Select "Clear Actions"
   - Change the entry under "Install on Target" to "install" by double clicking on "no action"
   
14. Enter the IP address of your Jetson TX2 system, as well as the username and password

## Installation on Jetson TX2
1. Install [Desktop-Full ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   - Type `chmod +x ros-kinetic-desktop-full-install.sh`
   - Type `./ros-kinetic-desktop-full-install.sh` to install Desktop-Full ROS Kinetic
   
2. Git clone this repository
   - Open your terminal
   - Go to the directory you wish to clone the repository in
   - Type `https://github.com/reuben-thomas/NP-DevCar.git`
   
3. Install the required packages
   - Type `chmod +x requirements.sh`
   - Type `./requirements.sh` 

## Driving the Servo
1. Open the Arduino IDE on the host computer

2. Configure the correct board and port on the IDE

3. Flash the scripts/Ardu.ino file into the Arduino Nano Every

4. Assemble the Arduino Nano Every

5. Run `roscore` on the Jetson TX2

6. Run the rosserial script
   - Type `rosrun rosserial_python serial_node.py /dev/<serial_port>`
   
7. Test the servo
   - Type `rostopic pub -r 10 car/cmd_vel geometry_msgs/Twist '{angular: {x: 0.0, y: 0.0, z: <angle>}}'`
