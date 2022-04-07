# OpenPodcar

Open source hardware design and software for OpenPodcar.

## Table of Contents
1. (General Info)(#general-info)
2. (Bill of Materials (BOM))(#bill-of-materials-(bom))
3. (Hardware Setup)(#hardware-setup)
4. (Software Setup)(#software-setup)
6. (User Guide)(#user-guide)
7. (General Testing)(#general-testing)
8. (Simulation)(#simulation)
9. (Troubleshooting Guide)(#troubleshooting-guide)

## General Info:
The OpenPodcar project includes:
- Hardware design and instruction to build and wire the physical car
- Arduino software for speed control
- Pololu software for steering control
- ROS drivers for whole vehicle steering and speed
- Gazebo simulation
- move_base and gmapping integration

To build the physical OpenPodcar, first obtain the components detailed in #bill-of-materials-(bom), then follow the steps of the build process detailed in (#hardware-setup), (#software-setup) and (#user-guide). The simulation can be directly installed in (#simulation).

## Bill of Materials (BOM)
Obtain the following components, which are available from many commercial suppliers and some weblinks are suggested.

### For the vehicle
		- 1 Shoprider mobility scooter
		- 1 3D Velodyne Lidar (VLP-16) with its controller box e.g. (https://velodynelidar.com/products/puck/)
		- 1 laptop under linux Ubuntu 16.04 e.g. (https://www.lenovo.com/gb/en/laptops/thinkpad/p-series/ThinkPad-P51/p/22TP2WPWP51)
		- 1 laptop under Windows 7+
		- 1 Gimson Robotics GLA750-P 12V DC linear actuator with position feedback (250mm stroke) e.g. (https://gimsonrobotics.co.uk/categories/linear-actuators/products/gla750-p-12v-dc-linear-actuator-with-position-feedback)
		- 1 ON/OF Toggle/Flick switch button e.g. (https://www.ebay.co.uk/itm/12V-HEAVY-DUTY-25A-UNIVERSAL-METAL-SPRING-MOMENTARY-ON-OFF-TOGGLE-FLICK-SWITCH/231257116468?epid=1229817686&hash=item35d7ff8f34:g:f50AAOSwq5lTmZKx)
		- 1 USB hub (with at least 3 USB ports) e.g. Amazon Basics (https://www.amazon.co.uk/AmazonBasics-4-Port-Ultra-Mini-Bus-powered/dp/B003M0NURK/ref=sr_1_3?keywords=amazonbasics%2Busb%2Bhub&qid=1649172163&sr=8-3&th=1)
		- 1 USB A cable (Arduino <-> Laptop)
		- 1 USB mini B cable (Pololu <-> Laptop)
		- 1 Ethernet cable (Velodyne <-> Laptop) 
		- 1 deadman push button for the vehicle ignition system e.g. (https://www.ebay.co.uk/itm/Philmore-30-825-SPST-Hand-Held-Push-Button-Switch-N-O/253746249505?epid=17011384611&hash=item3b14747f21:g:GdEAAOSwdA5bRopR)
		- 1 Joystick with its USB cable e.g. (https://www.logitechg.com/en-gb/products/space/extreme-3d-pro-joystick.942-000031.html)
		- 1 Manfrotto Black Digi Table Tripod 709B e.g. (https://www.manfrotto.com/ie-en/black-digi-table-tripod-709b/)
		- 1 Inline car fuse e.g. (https://www.ebay.co.uk/itm/BRAND-NEW-STANDARD-INLINE-CAR-FUSE-BLADE-HOLDER-30AMP-12V-SPLASH-PROOF-K265/170976134151?epid=23017011962&hash=item27cef8c407:g:NvoAAOSwtGNZzh9k)
		- 1 fuse 7.5Amp
		- 1 Relay e.g. (https://www.ebay.co.uk/itm/164139969972?hash=item2637812db4:g:LqAAAOSwX9VefSO4)
		- A dozen of bolts, nuts, washers and cap nuts

### For the PCB
		- 2 XL4016 Step-Down Buck converters DC-DC e.g. (https://www.amazon.co.uk/dp/B086W6B9H1/ref=pe_27063361_485629781_TE_item?th=1)
		- 1 Arduino Uno Rev3 e.g. (http://store.arduino.cc/products/arduino-uno-rev3)
		- 1 Pololu JRK 21v3 Motor Controller with Feedback e.g. (https://coolcomponents.co.uk/products/jrk-21v3-usb-motor-controller-with-feedback-fully-assembled?variant=45223123918&gclid=CjwKCAjwo_HdBRBjEiwAiPPXpNz4xgpL7U6n4qPT4Y1LRuD8ukLQVzoA5_KsBCFyvRQVbbDKt1zKYRoCGTQQAvD_BwE)
		- 1 MCP4725 DAC e.g. (https://protosupplies.com/product/mcp4725-12-bit-dac-module/)
		- 1 7-segment LCD with 3 digits e.g. (https://www.alibaba.com/product-detail/Taidacent-Red-0-200V-Measurement-Range_60679181399.html?spm=a2700.details.0.0.1f1212advD463M)
		- 1 Velodyne controller box (comes with the Velodyne lidar)
		- 1 10K Resistor
		- 1 100K Resistor
		- 3 Screw Terminal Block connectors (https://www.amazon.co.uk/Universal-Connector-Calculators-Electronic-Equipments/dp/B0922KLC4K/ref=sr_1_5?dchild=1&keywords=2+Pin+pinch+PCB+mount+screw+2.54&qid=1635870919&sr=8-5)
		- Male and female headers for Arduino and Pololu JRK 21v3 (https://www.amazon.co.uk/dp/B07CWSXY7P/ref=pe_27063361_485629781_TE_item)
		- A dozen of M1.5, M2.5 and M3 bolts 
		- A dozen of M1.5, M2.5 and M3 nuts 

### Other Useful Tools
		- Soldering iron
		- Solder
		- Gauge Wires 
		- Female and Male insulated electric connector crimp bullet terminal e.g. (https://www.aliexpress.com/item/32874324815.html)
		- Female and Male insulated crimp bullet terminal e.g. (https://www.ebay.co.uk/itm/333917138391?_trkparms=amclksrc%3DITM%26aid%3D1110006%26algo%3DHOMESPLICE.SIM%26ao%3D1%26asc%3D20200818143230%26meid%3Deba078cf394f4611b763dffe1c093587%26pid%3D101224%26rk%3D2%26rkt%3D5%26sd%3D264495178129%26itm%3D333917138391%26pmt%3D1%26noa%3D0%26pg%3D2047675%26algv%3DDefaultOrganicWeb%26brand%3DUnbranded&_trksid=p2047675.c101224.m-1&amdata=cksum%3A333917138391eba078cf394f4611b763dffe1c093587%7Cenc%3AAQAGAAAB8C%252Fyb1lwqzYRQPn6yDPD%252FrO6eM54Ozija9VIM4RJxPwnf1iY01i8i01AGZBZPkn9u2ICy0UZ8GznjGHI5H3qlDdJfGplg3%252FnVmb7jjFFUM4IVEm7R3DPDtNxLKKHo%252BEwplCvVFY6kBB7EzGGuHW3LSvzdrBOknWZFElCIFXC9wNLCvDkuoMgSJcUhIQeym%252BWUipLsT6%252FksKzu7uTbHIhtonSUjivOjZmnmISIZ7%252FiQuJR4%252BbEOz%252BftDUwR%252BY48cJQBva8gKVlgfC29kTN%252B3cdMZZEg3%252FH7hEKiajloCOYUJ5Vne%252FNz%252BxQOMHJxAbFdWkJlP4Ek1uctcOCFbrasOOehfastcE8yTbS8sqH%252BrGGdBZ2W1JAq2Gm7fTQQp%252FKFJwXtc13AYOV2A538ViL5eTdtrrWKw7qH03MghtN%252BDdRT1IGU26MRDd29X4oGcDbrxLcdSzbxwjw2nLfDhD2KaJUQO%252B%252FfjmEn9ItE6IdiS7v%252FNc%252Bsg%252BJy2sVHs8Bj7rosDy%252FU358Txs9Ynb8yOlXX70O%252B1WTb4eEuaaNbwHXY2VR065xgoEGG84ZXV8RU5RZ74kZURWNMb4zdAxab5OpJDJxF1lJ3XCa9J7JaQN%252BaLV2qnIdGfTAjQDfcrRl9%252Fu4jZg0BP7Wz66zqauiz9cxYBh1Wc%253D%7Campid%3APL_CLK%7Cclp%3A2047675)
		- Power supply 24V or more e.g. (https://www.amazon.co.uk/Lavolta-Variable-Linear-Bench-Supply/dp/B019KL4G6I/ref=sr_1_8?dchild=1&keywords=bench+power+supply&qid=1629281239&sr=8-8)
		- breadboard e.g. (https://www.amazon.co.uk/K-H-RH-32-Solderless-Breadboard/dp/B07DNB74FV/ref=sr_1_26?keywords=breadboard&qid=1649177149&sprefix=breadbo%2Caps%2C100&sr=8-26)
		- multimeter e.g. (https://www.machinemart.co.uk/p/clarke-cdm10c-digital-multimeter-5-function/)
		- clamp meter e.g. (https://www.tester.co.uk/uni-t-ut210b-clamp-meter)
		- Screwdrivers, pliers, automatic wire stripper etc.
		- Spare fuses e.g. (https://www.halfords.com/tools/fuses-electricals-and-fixings/fuses/)

## Hardware Setup

### Linear Actuator
	Before mounting the linear actuator under the vehicle, first test it with the Pololu to check that it is functional.

#### Initial Testing
		This section shows how to test the linear actuactor before mounting it.
		- Material: Gimson linear actuator, Pololu JRK 21v3 Configuration, external power capable of supplying 12V, a breadboard, some wires, a multimeter
		- Start testing the linear actuator as follows:
			- Wire the power supply **-** to Pololu's **GND**
			- Wire the power supply **+**  to Pololu's **VIN** 
			- Wire the linear actuator's **black wire** to Pololu's *A*
			- Wire the linear actuator's **red wire** to Pololu's *B*
			- Use a breadboard to make the following connections:
				- Wire the linear actuator's **blue wire** to Pololu's *FB* 
				- Wire the linear actuator's **yellow wire** to Pololu's *+5V* below **FB**  [# Check]
				- Wire the linear actuator's **white wire** to Pololu's *GND* below **+5V**  [# Check]
			- Set the external power to supply 12V -> the Pololu LED should start blinking orange
			- Connect the Pololu USB to a computer with the "Pololu JRK 21v3 Configuration" Tool
			- Open the Configuration Tool Interface and go to the **Error** tab to check that no error is displayed other than the flag waiting for new commands. 
				- If there are errors, click on "**Reset**" and "**Clear**" until the Poolu's LED start blinking green
				- Go to **Input** tab and use the cursor to send commands to the linear actuator e.g.
					- "2500" : the linear actuator should extend its length to maximum
					- "1500" : the linear actuor should reduce its length
					- "1900" : the linear actuator should have a medium length. NB: this is the position that the linear actuator should be when mounting it underneath the vehicle 	
		
#### Mounting
		- Material: Gimson linear actuator, a drill, some bearings
		- To access the underside of the vehicle, this requires the help of at least three people:
			- Bring in two axle stands as high as 75cm each
			- Place the two axle stands about one meter away from the front and back wheels, both in the side of the vehicle, as shown in the picture !(Vehicle tilted using on axles)(docs/hardware/onAxles.jpg)
			- Place two jacks right next to the front and back wheels, on the same side as the axle stands cf. the previous picture
			- Two people, one person should stand next to each axle stand
			- Another person should stand on the other side of the vehicle and push the vehicle from the top towards the people next to the axle stands
			- The persons next to the axle stands should adjust the positions of the stands in order to get the vehicle nicely tilted without any instability
			- The vehicle should now be tilted on one side and its underside clearly visible in order to mount the linear actuactor
		- To mount the linear actuactor:
			- The linear actuator must have a medium length obtained via the Pololu command "1900", cf. the Pololu Configuratin steps detailed above. 
			- There is an existing hole in the right front wheel axle where the front hole of the linear actuator is fixed with a bolt, a nut, several washers and a cap nut [# check]
			- A hole was drilled to the left side of the front chassis to fix the back of the linear actuactor via bearings [# check]
			- The final mounting is shown in this picture !(Vehicle tilted using on axles)(docs/hardware/steeringActuatorMounted.jpg)
			
	Once the mounting is finished, the three people should bring the podcar back to its normal position, remove the axle stands and jacks before making a final test with the linear actuactor.

#### Final Testing
			- Repeat the **Initial Testing** protocol presented above, but this time with the linear actuactor mounted under the vehicle, its wires can be pass through an empty area between the plastic bumper and the vehicle left battery. This final test will helps to verify that the linear actuactor is well mounted and can steer the wheels as expected:
				- a "2500" command should steer the front wheel to the far right, i.e about -45 deg
				- a "1900" command should keep the front wheels forward facing, i.e. about 0 deg
				- a "1500" command should steer the front wheels to the far left, i.e. about +45 deg
		
### Fusing and ON/OFF Switch
	- Material: inline car fuse, fuse 7.5Amp, ON/OFF switch, x2 12V batteries (the ones in the vehicle, under the seatback), some wires
		- Fix a tick wire using a crimp bullet terminal onto each pin of the switch [# check]
		- Get access to the right battery's **+** pole and use a plier to remove the nut [# check]
		- Strip the wire on the **+** pin of the switch by 3cm and connect it to the **+** of the right battery by tangling it around the battery **+** pole
		- Fix back the nut that was removed from the battery **+** pole
		- Connect the switch **-** pin to the fuse **-** wire
		- Extend the fuse **+** wire by 30cm e.g. solder another tick wire to it, then keep it safely for later connection with the buck converter **+** on the PCB board
		- Get access to the left battery's **-** pole's wire, strip a small area in the middle and plug a new and long wire (~50cm) there, then keep this new wire safely for later connection with the buck converter **-** on the PCB board


### 3D Lidar
		- The lidar screws onto the tripod. The tripod is cabled-tied to the vehicle roof via  drilled holes at locations in Fig. TODO.   But it needs to be set up so we can talk to the Velodyne over Ethernet. The laptop must be on a wired network, not wifi. The IPs must be configured as in the velodyne. The default lidar IP is 192.168.1.201.

		- RUNNING : must be on wired ethernet -- not wifi (can jump to wifi sometimes?)
			```
			roslaunch velodyne_pointcloud VLP16_points.launch
			rostopic echo /velodyne_points
			rosrun rviz rviz -f velodyne
			```
		- In Rviz "displays" panel, click "Add", then select "Point Cloud2", then press "OK".
		- In the "Topic" field of the new "Point Cloud2" tab, enter "/velodyne_points".


### Printed Circuit Board (PCB)

Manufacture the PCB board: this can be done by sending the gerber files to an online PCB manufacturer such as (https://www.pcbway.com). They will then post the bare board to you. At this stage there are no components on it, you will solder them on later in these instructions.
	
#### Initial Testing
		- Material: the manufactured PCB board, a multimer
		- Simply use the multimeter in continuity mode to check whether any of the PCB connections is broken. If a connection on the board is continuous i.e. fine, then the multimeter emits a continuous **bip**

#### Buck Converter Settings
		- XL4016 Buck converter 1
			- set output voltage to 16V (or the voltage required for your laptop)
			- Set output current to 3.75A (or the maximum current required for your laptop)
		- XL4016 Buck converter 2
			- set output voltage to 12V (or the voltage required for the Pololu and 3D Lidar)
			- set output current to 3A (or the maximum current required for the linear actuator and 3D Lidar)
		- **Testing**
			- Material: buck convertes, power supply, multimeter and some wires
			- Use a power supply


#### Arduino Software
		- Download the MCP4725 library into Arduino's LIBRARIES
		- Upload the Arduino Code
		- **Testing**
			- Material: Arduino
			- Open the serial monitor to send some commands e.g. "FA:170" for zeroing, "FA:210" for forward speed, "FA:120" for reverse speed
			
#### Pololu JRK 21v3 Configuration
			- Download Pololu's Windows Configuration Tool by following these steps: (https://www.pololu.com/docs/0J38/3.a)
			- To configure the Pololu, please follow the instructions here: (https://www.pololu.com/docs/0J38/5)
			- For OpenPodCar, we followed the steps below:
				- Connect the USB to Pololu and open "Pololu Jrk COnfiguration Utility" tool
				- Go to "**Error**" tab:
					- set "**No power**", "**Motor driver error**", "**Feedback disconnected**" and "**Max. current exceeded**" to "**Enabled and latched**"
					- click on "**Apply settings to**"

				- Go to "**Input**" tab: 
					- set "**Input mode**" to "**serial**"
					- keep all other parameters to default values
					- Final settings should like here: !(Pololu Input tab)(/docs/software/pololu_input.png)

				- Go to "**Feedback**" tab:
					- set "**Feedback mode**" to "**Analog voltage**"
					- check "**Invert feedback direction**"
					- Calibration:
					- set "**Absolute Max**" to "**2600**"
					- set "**Maximum**" to "**2600**"
					- set "**Minimum**" to "**1000**"
					- set "**Absolute Min**" to "**1000**"
					- keep all other paramters to default values
					- Final settings should like here: !(Pololu Feedback tab)(/docs/software/pololu_feedback.png)

				- Go to "**PID**" tab:
					- set "**Proportional Coefficient**" to "**6**" at the top and "**1**" at the bottom, so that to get a final "**3**"
					- keep all other parameters to default values 
					- Final settings should like here: !(Pololu PID tab)(/docs/software/pololu_pid.png)

				- Go to "**Motor**" tab:
					- check "**Invert motor direct**"
					- in the "**Forward column**", set "**Max. current (A)**" to "**0**"
					- keep all other parameters to default values
					- Final settings should like here: !(Pololu Motor tab)(/docs/software/pololu_motor.png)

				- Click on "**Apply settings to**"
			
			
#### MCP4725 DAC
		- Solder the male headers provided for the DAC pins.
		- **Testing**
			- Material: MCP4725 DAC, Arduino, some wires, multimeter
			- Use a breadboard to make the following connections:
				- Wire both DAC **GND** to both Arduino **GND**, using two female to male wires
				- Wire the DAC **VCC** to Arduino **5V**, using a female to female wire
				- Wire the DAC **SDA** to Arduino **SDA**, using a female to male wire
				- Wire the DAC **SCL** to Arduino **SCL**, using a female to male wire
				- Wire the DAC **OUT** to an isolated point on the breadboard, using a female to female wire
			- Turn on the Arduino by connecting it via USB to a computer 
			- Take the multimeter to measure the voltage received on the DAC:
				- Set the COM 
				- Voltage between the DAC **GND** and **VCC** should give a value between **4.7V** to **5V** i.e. equivalent to Arduino input voltage
				- Voltage between the DAC **GND** and **OUT** should give a value between **2.1V** and **2.4V**
			
#### Assembly
		- 
			
#### Final Testing
	The PCB board was heavily tested before and after assembling its components to ensure that once it is integrated into the vehicle, there would not be any big issue. For instance, in Fig. \ref{fig:pcb_testing}, we used an external power supply and a multi-meter to measure the voltage across the PCB components, check the safety of the board and ensure that the components work as expected.
	- Material: power supply, some wires, a multimeter	
	
		
### Vehicle Connections
	

#### Connect the linear actuactor to the Pololu JRK 21v3

#### Connect the 3D lidar to the PCB 


#### Deadman handle (DMH) and Relay

	The addition of the Relay and the DMH Switch are essential for safe operation, especially where new unproven autonomous control systems are in development.
	A two stage approach is used to reduce this risk. Refer to the schematic diagram DMH section in conjunction with this description.

	\paragraph{Stage 1 – Relay.} A relay is used which interrupts the mobility scooter’s key ignition circuit. If the relay is not energised by the presence of a 5V supply to the Arduino, the vehicle’s movement is disabled. This effectively ensures that if the Arduino is non-functional, for example its power supply has failed or it has been unplugged from the USB port of the control PC and there is a danger that the DAC is not producing the control systems required voltage, the scooter is automatically disabled by effectively switching it off.

	\paragraph{Stage 2 – DMH Switch.} A sturdy push button is used which also interrupts the vehicle's key ignition circuit. If the Podcar operator detects any abnormality in operation during operation, he/she simply releases pressure from the DMH switch and the vehicle’s movement is disabled. The DMH switch is wired in series with the relay in the key ignition circuit ensuring that if both the relay contacts and the DMH switch are closed, this is the only condition where the Podcar movement is active.
	

#### Connect the PCB components to DMH and Relay
	
	


## Software Setup

### Arduino
	- Arduino firmware source is supplied in the distribution ThrottleControlSerial.ino

### Velodyne 3D Lidar Configuration with ROS
	- Please follow the instructions from here: (http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
	
	
### Simlink USB COM Ports
	Here, the goal is to create persistent USB serial device names (aka SIMLINK) for the Arduino and the Pololu.
	- In terminal: `sudo nano /etc/udev/rules.d/99-tty.rules`
	- Connect the Arduino and the pololu via USB to your laptop, then type in the terminal:
		```
		sudo lsusb -v | grep 'idVendor\|idProduct\|iSerial'
		```
	- The above command displays the idVendor and idProduct for all the serial devices connected to your laptop and needs to be reported in the Simlink. 
	- For example, for the OpenPodcar we copy the following lines in "99-tty.rules":
		```
		#Arduino com-port rules
		SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="ttyArduino", GROUP="dialout", MODE="0666"
		#Pololu com-port rules -- NB the pololu has two virtual serial ports on a single USB interface.
		SUBSYSTEM=="tty", ATTRS{idVendor} =="1ffb", ENV{ID_USB_INTERFACE_NUM}=="00"  SYMLINK+="ttyPololuCOM"
		SUBSYSTEM=="tty", ATTRS{idVendor} =="1ffb", ENV{ID_USB_INTERFACE_NUM}=="02"  SYMLINK+="ttyPololuTTL"
		```
	- Save and close `99-tty.rules' file
	- Then type in the terminal `sudo reboot`
	- Check the status of each SIMLINK by typing in terminal for example: `ls -l /dev/ttyArduino` or `ls -l /dev/ttyPololuCOM`
		
### Object Detection and Tracking
	- Install the FLOBOT project: please follow the "Install & Build" guide here: (https://github.com/LCAS/FLOBOT)
	- BSON for Python is required, this can be installed via ``` pip install pymongo==3.5.1 ```
	- Open flobot_tracker.launch and change frame names and paths to the ones your system uses
	

	

## User Guide 

- Check that the vehicle’s original lever for auto-manual is set to auto (DOWN). It is on the main motor, under the vehicle at the rear left, colored red. Requires some force to move it.

- Power on the vehicle using the original on-off switch located under the seat on the left. It is marked ON-OFF.

- Power on the modified electronics using the new toggle switch.   (This lights LEDs on the DCDCs and Pololu, and the lidar makes a whirring sound).

- Check that the batteries are charged (use a multimeter across one of the DCDC converters, need to see 24V or over. {\em Do not use the vehicle if it is undercharged, this is dangerous.}

- Power on the laptop using the slider switch on its front right.

- Login as user podcar, password TODO.

- Type:  roscd podcar

- Unplug the laptop’s USB connection and plug it in again.  (HACK) (not really needed ?!)

- Run the test script podcar/tools/pololuCSerialTest/a.out  (HACK) (not really needed ?!)

- Type: roslaunch podcar podcar.launch

- Use the joystick to control steering and speed for manual drive.

- For automation, the steps should be exactly the same as above but with the automation launch file.


## General Testing

	- Speed control dead zone
	%move to git:
	Implementing and testing this safety system should be undertaken with the drive wheels of the vehicle raised off of the ground, allowing for checks to be made of the DMH without the risk of the vehicle speeding off out of control.



 - Steering Control
 
 
 - GMapping
 
 
 - Move_base and TEB planner



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

%Fig. \ref{fig:sim_nodes} shows the complete ROS node configuration used during simulation, under manual joystick control.
%\begin{figure}(h)
%	\includegraphics(width=\columnwidth){figs_sim/sim_nodes.png}
%	\caption{ROS nodes used in simulation under manual joystick control.}
%	\label{fig:sim_nodes}
%\end{figure}

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
	
- Linear Actuator
	- Diagnostic test commands can be passed to the Polulo using the commands provided in /tools/cmdSteer.   {\em Do not give commands outside the range 1000-2500 as they have mechanically destroyed the the vehicle.} A non-ROS test of the C API for the Pololu is provided in /tools/pololuTestCSerial.
	
- Speed
	- It receives commands of the form “FA:210” as speed commands. The test scripts /tools/zeroSpeed.py and /tools/testSpeed.py can be used to send example commands for debugging.
	
- SIMLINK
	- If the Simlink does not work, display all the devices by typing in terminal `ls -l /dev` to see whether your device is connected well.
	- The idVendor and idProduct can also be displayed for each USB device, such as `udevadm info -q all -a -n /dev/ACM0` for the device connected at COM Port ACM0

### Lidar
- No velodyne_points message published
	- Check: laptop must be on wired network, not wifi.

	- Check: wired network must be configured correctly, see velodyne setup docs. Maybe be interfered if wifi has been used recently.
	
	- Check connections to Velodyne box including power and ethernet.
   

### Simulation
- AF\_NET error
	- If this is thrown by the Gazebo plugin -- it may be because Gazebo is being run standalone rather than launched as a ROS node as required.
	
- Cannot locate node i.e. you should make your Python scripts as executables in order to launch them



Disclaimer: Neither the authors nor the University of Lincoln are repsonsible for accidents, injuries or damage caused by this vehicle design, and by downloading, building or operating the design you agree to do so entirely at your own risk.  The design is not a legal product and carries no safety certification.

# Licence [# check]

