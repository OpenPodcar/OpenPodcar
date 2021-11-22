# podcar

(Work in progress not ready for public use)

Open source hardware design and software for Lincoln Podcar.

Including:

- hardware design and instruction to build and wire the physical car
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


### Buck converters' settings
- Set buck converter 1's output voltage to 16V
- Set buck converter 2's output voltage to 12V


### Arduino Software
- Download the MCP4725 library into Arduino's LIBRARIES
- Upload the Arduino Code


### Pololu Configuration
- Download Pololu's Windows Configuration Tool, follow these steps: [CITE LINK]
- Configure the Pololu:
-- Input tab:
-- Motor:
-- PID:
-- Error:


## Simulation installation

Assuming: ros kinetic and gazebo 7 already installed. (Install instructions can be found here: http://wiki.ros.org/kinetic/Installation)

Rosdep is also required, but is standalone since ROS Fuerte. Install instructions can be found here: http://wiki.ros.org/rosdep

To install the ros package and gazebo sim locally, clone the repository and run the follow commands (directories indicated by <> need to be replaced by their actual locations):

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

Once the simulation is running, you can then launch one of two different systems to control the robot: manual joystick control or movebase control.


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
