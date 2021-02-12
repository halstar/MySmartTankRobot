# MySmartTankRobot

A homemade smart tank robot built on a Raspberry Pi 3 B+ basis, with DC motors, micro servo motors, different regular sensors (IMU, LIDARs) and an infrared camera.

This repository gathers sources Python scripts to be executed on the Raspberry Pi.

You can see the result at work with this playlist:

https://www.youtube.com/playlist?list=PLDA98d1k4czB_mNHSkv1oVTqYg_OaIa2p

**How to setup the Raspberry Pi for this project**

* From a fresh install, update the whole Pi:

	* sudo apt-get update
	* sudo apt-get upgrade
	
* Install Open Computer Vision library for Python:	

	* sudo apt-get install liblas-dev libatlas-base-dev libjasper-dev libqtgui4 libqt4-test 
	* sudo pip3 install python3-picamera opencv-python opencv-contrib-python
	* export LD_PRELOAD=/usr/lib/arm-linux-gnueabihf/libatomic.so.1
	
* Install VL53L1X (LIDAR) library for Python:

	* sudo pip3 install vl53l1x
	
* Install PIGPIO library for Python:

	sudo pip3 install pigpio

*  Install smbus2 I2C library for Python:

	* sudo pip3 install smbus2

* Setup I2C bus to its maximum speed:
	
	* sudo vi /boot/config.txt
	* dtparam=i2c_arm=on,i2c_arm_baudrate=400000
	* sudo reboot

* Manually start PIGPIO daemon (add this to rc.local or .bashrc or other):

	* sudo pigpiod

**Note**:  so, pigpio is used for GPIO access. Though they are available/activable in this project, RPi.GPIO & gpiozero supports were only partially tested.