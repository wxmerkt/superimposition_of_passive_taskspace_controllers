# Stack of Task Space Controllers Framework [![Build Status](https://travis-ci.org/wxmerkt/stack_of_passive_controllers_controller.svg?branch=master)](https://travis-ci.org/wxmerkt/stack_of_passive_controllers_controller)

This framework implements the *Stack of Task Space Controllers* concept introduced in:

> Carlo Tiseo*, Wolfgang Merkt*, Wouter Wolfslag, Sethu Vijayakumar, and Michael Mistry. Safe and Compliant Control of Redundant Robots Using a Stack of Passive Task-Space Controllers. [arXiv: 2002.12249]([abc](https://arxiv.org/abs/2002.12249))

It is split in several packages:

1. `stack_of_task_space_controllers_core` which implements the core library for the stack of controllers and the passive Fractal Impedance Controller described in the above paper.
2. `stack_of_task_space_controllers_ros_control` which provides a ROS-Control controller.
3. `stack_of_task_space_controllers_kuka_lwr` which includes a Gazebo simulation using the ROS-Control controller as well as a node communicating with the IPAB-SLMC `lwr_driver` for hardware experiments.
4. `stack_of_task_space_controllers_rocoma` which wraps the framework for use with the RoCo control framework.
