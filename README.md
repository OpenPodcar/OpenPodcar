# OpenPodcar

(Work in progress not ready for public use)

Open source hardware design and software for Lincoln Podcar.

Including:

- Hardware design and instruction to build and wire the physical car
- Arduino software for speed control
- Pololu software for steering control
- ROS drivers for whole vehicle steering and speed
- Gazebo simulation
- move_base and gmapping integration

Disclaimer: Neither the authors nor the University of Lincoln are repsonsible for accidents, injuries or damage caused by this vehicle design, and by downloading, building or operating the design you agree to do so entirely at your own risk.  The design is not a legal product and carries no safety certification.


## Bill of Materials (BOM)

	- 1 Shoprider mobility scooter
	- 1 3D Velodyne Lidar (VLP-16) with its controller box
	- 1 laptop with Ubuntu 16.04 and ROS Kinetic and Gazebo 7 installed
	- 1 Gimson Robotics linear actuator with position feedback (250mm stroke)
	- 1 switch button
	- 1 Amazon USB hub (with at least 3 USB ports)
	- 1 USB A cable (Arduino <-> Laptop)
	- 1 USB mini B cable (Pololu <-> Laptop)
	- 1 Ethernet cable (Velodyne <-> Laptop) 
	- 1 deadman button for the vehicle ignition system
	- 1 Joystick with its USB cable

### BOM for the PCB 
	- 2 XL4016 Step-Down Buck converters 
	- 1 Arduino Uno Rev3
	- 1 Pololu JRK 21v3 Motor Controller with Feedback
	- 1 MCP4725 DAC 
	- 1 7-segment LCD with 3 digits
	- Velodyne controller box
	- 1 10K Resistor
	- 1 100K Resistor
	- 3 connectors
	- Male and female headers for Arduino, Pololu and DAC 
	- M1.5, M2.5 and M3 bolts
	- M1.5, M2.5 and M3 nuts


### Useful Tools
	- Soldering iron
	- Solder
	- Power supply
	- Screwdrivers, pliers etc.



## Physical Podcar Setup


- **Buck converters' settings**
	- Buck converter 1
		- set output voltage to 16V (or the voltage required for your laptop)
		- Set output current to 3.75A (or the maximum current required for your laptop)
	- Buck converter 2
		- set output voltage to 12V (or the voltage required for the Pololu and 3D Lidar)
		- set output current to 3A (or the maximum current required for the linear actuator and 3D Lidar)


- **Arduino Software**
	- Download the MCP4725 library into Arduino's LIBRARIES
	- Upload the Arduino Code


- **Pololu JRK 21v3 Configuration**
	- Download Pololu's Windows Configuration Tool by following these steps: [https://www.pololu.com/docs/0J38/3.a]
	- To configure the Pololu, please follow the instructions here: [https://www.pololu.com/docs/0J38/5]
	- For OpenPodCar, we followed the steps below:
		- Connect the USB to Pololu and open "Pololu Jrk COnfiguration Utility" tool
		- Go to "**Error**" tab:
			- set "**No power**", "**Motor driver error**", "**Feedback disconnected**" and "**Max. current exceeded**" to "**Enabled and latched**"
			- click on "**Apply settings to**"

			- wire the Pololu to power (VIN and GND)
			- wire the Pololu's motor outputs "A" and "B" to the linear actuator 
			- wire the Pololu's feedback pins to the linear actuator feedback wires
			- turn the power on
			- click on "**Reset**" and "**Clear**" in the "**Error**" tab
			- the yellow LED on the Pololu should start blinking

		- Go to "**Input**" tab: 
			- set "**Input mode**" to "**serial**"
			- keep all other parameters to default values

		- Go to "**Feedback**" tab:
			- set "**Feedback mode**" to "**Analog voltage**"
			- check "**Invert feedback direction**"
			- Calibration:
			- set "**Absolute Max**" to "**2600**"
			- set "**Maximum**" to "**2600**"
			- set "**Minimum**" to "**1000**"
			- set "**Absolute Min**" to "**1000**"
			- keep all other paramters to default values

		- Go to "**PID**" tab:
			- set "**Proportional Coefficient**" as "**6**" at the top and "**1**" at the bottom, so that to get a final "**3**"
			- keep all other parameters to default values

		- Go to "**Motor**" tab:
			- check "**Invert motor direct**"
			- in the "**Forward column**", set "**Max. current (A)**" as "**0**"
			- keep all other parameters to default values

		- Click on "**Apply settings to**"


- **Velodyne 3D Lidar Configuration with ROS**
	- Please follow the instructions here: [http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16]

## Simulation Installation

- Assuming: ros kinetic and gazebo 7 already installed. (Install instructions can be found here: http://wiki.ros.org/kinetic/Installation)

- Rosdep is also required, but is standalone since ROS Fuerte. Install instructions can be found here: http://wiki.ros.org/rosdep

- To install the ros package and gazebo sim locally, clone the repository and run the follow commands (directories indicated by <> need to be replaced by their actual locations):

```bash
cd <install location>/catkin_ws
catkin_make
source devel/setup.bash
cd src/podcar/models/plugins
cmake .
make
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH`pwd`:
cd ..
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH`pwd`:
cd ../../../
rosdep install --from-paths src --ignore-src -r -y
roslaunch podcar podcarsim.launch
```

The first run of Gazebo may take a while (e.g. 5 minutes) to load because models used need to be automatically downloaded from the remote gazebo repositories.

- Once the simulation is running, you can then launch one of two different systems to control the robot: manual joystick control or movebase control.


### Joystick control

If you have a USB joystick connected, open a new terminal and run,

```bash
source <install location>/catkin_ws/devel/setup.bash
roslaunch podcar joystick.launch
```

### Move_base control

Open a new terminal and run,

```bash
source <install location>/catkin_ws/devel/setup.bash
roslaunch podcar podcarsim_moveBase.launch
```

This will present a standard movebase GUI interface in rviz, enabling you to click desired destinations to command the vehicle to drive to.

## Troubleshooting Guide

### Vehicle
- vehicle beeps continuous when press DMH and rear wheels do not move
	- This is due to a safety mode preventing ignition.

	- Check: is the manual-auto switch under the rear motor on auto?

	- Check: are the batteries well charged (must be 24V or over.) ?

	- Check: is the control voltage is the dead zone ? It should be.

- Rear wheels do not move, control voltages are correct}
	- Control voltages means the display on the voltmeter LED. Should be above 1.2 or below 1.8 for forward and backwards.

	- Check: main vehicle battery level, by connecting the vehicle charger and inspecting the battery charge level. Problem occurs if the battery is nearly flat.

	- Check: charger must be disconnected for rear wheels to move (safety feature).

### Lidar
- No velodyne_points message published
	- Check: laptop must be on wired network, not wifi.

	- Check: wired network must be configured correctly, see velodyne setup docs. Maybe be interfered if wifi has been used recently.
	
	- Check connections to Velodyne box including power and ethernet.
   

### Simulation
- AF\_NET error
	- If this is thrown by the Gazebo plugin -- it may be because Gazebo is being run standalone rather than launched as a ROS node as required.
