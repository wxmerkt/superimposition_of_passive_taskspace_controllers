# stack_of_passive_controllers_controller [![Build Status](https://travis-ci.org/wxmerkt/stack_of_passive_controllers_controller.svg?branch=master)](https://travis-ci.org/wxmerkt/stack_of_passive_controllers_controller)

ROS-Control controller implementing a Stack of Passive Cartesian Controllers

## Kuka LWR3+ Real Experiment
1. `roscore`
2. `roslaunch lwr_driver driver.launch`
3. Launch `slmc_333` on Kuka
4. `rqt` (for Kuka UI, load the UI plugin)
5. `roslaunch stack_of_passive_controllers_controller load_lwr_robot_description.launch`
6. `rosrun stack_of_passive_controllers_controller go_to_start` (in position control mode)
7. `roslaunch stack_of_passive_controllers_controller lwr_fic_node.launch`
8. Change to Joint Stiffness Control

6. `roslaunch stack_of_passive_controllers_controller lwr_ik.launch` - will start IK and rviz
8. Go to the `rqt_reconfigure` to tune the gains; 