# podcar

Simulation of Shoprider Traveso mobility scooter used as automated podcar.


softlinked the model as
ln -s path/to/here/models/podcar to ~/.gazebo/models/podcar
(or can add extra model path in gazebo itself from the models gui)
                                                                     

In gazebo, need to manual set PID params to 1,0,.5 for front wheels.

Troubleshooting:
AF_NET error:
If this is coming from a gazebo plugin -- it may be because gazebo is being run standalone rather than launched from ros as required.


TODO: implement a standard twist interface to accept messages such as,
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

