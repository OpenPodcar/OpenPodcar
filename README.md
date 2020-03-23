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

(assumes ros kinetic already installed and gazebo 7)

cd <install location>/catkin_ws
catkin_make
source devel/setup.bash
cd src/podcar/model/plugin
cmake . ; make
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH`pwd`
cd ..
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH`pwd`
source /usr/share/gazebo/setup.sh
