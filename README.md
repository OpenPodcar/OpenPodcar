# podcar

(Work in progress not ready for public use)

Open source hardware design and software for Lincoln Podcar.

Including:

- hardware design and instruction to build and wire the physical car
- Arduino software for speed control
- Pololu software for steering control
- ROS drivers for whole vehicle steering and speed
- Gazebo simulation

Disclaimer: Neither the authors nor the University of Lincoln are repsonsible for accidents, injuries or damage caused by this vehicle design, and by downloading, building or operating the design you agree to do so entirely at your own risk.  The design is not a legal product and carries no safety certification.

To install the ros package and gazebo sim locally, clone/fork the repository and run the follow commands:

(assumes ros kinetic and gazebo 7 already installed, directories indicated by <> need to be replaced by their actual locations)

```bash
cd <install location>/catkin_ws
catkin_make
source devel/setup.bash
cd src/podcar/models/plugins
cmake . ; make
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH`pwd`:
cd ..
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH`pwd`:
```

You'll need to run the following command in every new terminal (alternatively, you can source this file in ~/.bashrc, but be sure to use the absolute path):
```bash
source <install location>/catkin_ws/devel/setup.bash
```

To run the simulation, use:
```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH`pwd`: #Not needed if this has already been run in the terminal
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH`pwd`: #Also not needed if this has already been run in the terminal
roslaunch podcar podcarsim.launch
```
The first run of this may take a while to load; this is because models used need to be downloaded from the gazebo (this is being done automatically).

To control the robot through a joystick, first connect a joystick and run (in a new terminal):
``` bash
roslaunch podcar joystick.launch
```

To control the robot using move_base, run (in a new terminal):
```bash
roslaunch podcar podcarsim_movebase.launch
```

