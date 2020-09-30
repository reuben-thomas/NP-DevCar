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

2. [JetpPack 3.3](https://developer.nvidia.com/embedded/jetpack-3_3)

3. [Orbitty L4T R28.2X BSP](https://connecttech.com/ftp/Drivers/CTI-L4T-V121.tgz)

###
1. [Desktop-Full ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

2. [Python 2.7](https://www.python.org/download/releases/2.7/)
   - [pip](https://pypi.org/project/pip/)
   - [rospy](http://wiki.ros.org/rospy)
   - [NumPy](https://pypi.org/project/numpy/)
   
3. [Git](https://git-scm.com/download/linux)

## Usage
`rosrun rosserial_python serial_node.py /dev/ttyACM0`
