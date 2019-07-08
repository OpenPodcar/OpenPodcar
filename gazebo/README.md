# OpenPodCar

Simulation of OpenPodCar open hardware vehicle.

To use in gazebo:

Start Gazebo
Open "Insert" tab
Click "Add path"
Add directory /path/to/podcar/gazebo/models
(Podcar should appear in model list.)
Click Podcar. Click on a ground location to instantiate.

(do not softlinks to ~/.models , the above way is neater)


                                                                     

In gazebo, need to manual set PID params to 1,0,.5 for front wheels.



TODO: implement a standard twist interface to accept messages such as,
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

