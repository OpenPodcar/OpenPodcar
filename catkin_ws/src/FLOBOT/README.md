# FLOBOT

[FLOBOT](http://www.flobot.eu/) is an EU funded Horizon 2020 project. The University of Lincoln is responsible for developing the human tracking for safety software module, and leading the environment reasoning and learning software module. In this repository, you find these two modules as a whole, and specifically:

* [bayestracking](bayestracking) is a library originally developed by [Dr. Nicola Bellotto](https://lncn.ac/nbellotto).

* [detector_msg_to_pose_array](detector_msg_to_pose_array), [ground_plane_estimation](ground_plane_estimation), and [upper_body_detector](upper_body_detector) was originally developed by [STRANDS](http://strands.acin.tuwien.ac.at/) project.

* [bayes_people_tracker](bayes_people_tracker) is also from [STRANDS](http://strands.acin.tuwien.ac.at/) but adapted according to the [FLOBOT](http://www.flobot.eu/) needs.

* [flobot_tracker_bringup](flobot_tracker_bringup), [object3d_detector](object3d_detector), [pose_grid_map](pose_grid_map), and [rgbd_leg_detector](rgbd_leg_detector) are originally developed by [FLOBOT](http://www.flobot.eu/).

A detailed view of the proposed system in UML diagram is shown below:

![flobot_uol_uml.jpg](https://github.com/LCAS/FLOBOT/blob/master/flobot_uol_uml.jpg)

In which `leg_detector` (and `people_msgs`) can be found here: [https://github.com/wg-perception/people](https://github.com/wg-perception/people). For a more intuitive understanding, please refer to the following video.

[![YouTube](https://img.youtube.com/vi/H2dBDKZMFTE/0.jpg)](https://www.youtube.com/watch?v=H2dBDKZMFTE)

## Install & Build

Our system has been tested on Ubuntu 14.04 + ROS Indigo, as well as Ubuntu 16.04 + ROS Kinetic. For the latter:

```bash
$ sudo apt-get install ros-kinetic-people*
$ sudo apt-get install ros-kinetic-velodyne*
$ cd catkin_ws/src
$ git clone https://github.com/LCAS/FLOBOT
$ cd ../catkin_ws
$ catkin_make
```

## Run
```bash
$ roslaunch flobot_tracker_bringup flobot_tracker.launch
```

## FLOBOT Perception Dataset

[http://lcas.github.io/FLOBOT/](http://lcas.github.io/FLOBOT/)

## Citation
If you are considering using this repository, please reference the following:

```
@article{zhimon2020,
  author  = {Z. Yan and S. Schreiberhuber and G. Halmetschlager and T. Duckett and M. Vincze and N. Bellotto},
  title   = {Robot perception of static and dynamic objects with an autonomous floor scrubber},
  journal = {Intelligent Service Robotics},
  year    = {2020},
  volume  = {13},
  number  = {3},
  pages   = {403-417},
  doi     = {https://doi.org/10.1007/s11370-020-00324-9}
}
```

## Related development

* [https://github.com/LCAS/online_learning](https://github.com/LCAS/online_learning)
* [https://github.com/LCAS/cloud_annotation_tool](https://github.com/LCAS/cloud_annotation_tool)
