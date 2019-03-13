\mainpage .

# Gravity Compensation Controller Tutorial

This package shows the user how to implement a gravity compensation controller by using the ros_control interface.

In this package the user could learn how to:

- Implement an effort controller that overcomes gravity.
- Create his own controller with the ros_control API.

## Usage

First install or deploy the specific package on your robot.

Then use the controller manager to stop all the active controllers that share resources with this controller. for example let's stop arm_controller
```
$ rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'arm_controller'
strictness: 0" 

```
If the output is true, this means that the controller stopped successfully. You cna either check by running:
```
$ rosservice call /controller_manager/list_controllers

```
This shows all the loaded controllers, and describes which ones are active and the resources claimed by every controller. No other controller should started should claim the same resources that our gravity_compensation_tutorial controller.

To start it run:
```
$ roslaunch gravity_compensation_controller_tutorial gravity_compensation_controller_tutorial.launch robot:=robot_used end_effector:=end_effector_used
```
This launch file contains two arguments:

- **robot** : By default tiago. Is the robot where we launch this controller.
- **end_effector** : By default is pal-gripper. End effector mounted on the robot. There are three options: pal-gripper, pal-hey4, schunk-wsg.

This launch file loads the specific parameters from the config files, loads the controller and starts it. All of this could be done manually by loading the specific parameters and then call the controller_manager services to load and start the controller.

Once finished **don't close the launch file. otherwise the arm will fall down!**. First switch to the arm_controller again.
```
$ rosservice call /controller_manager/switch_controller "start_controllers:
- 'arm_controller'
stop_controllers:
- 'gravity_compensation_tutorial'
strictness: 0" 

```
Then, once it is controlling in position mode again, the launch file could be canceled.

**NOTE:** This tutorial is supposed to be launched on the TIAGo Robot from **PAL Robotics**. If the user has his own gripper or wants to extrapolate it to another robot, it will be necessary to regenerate the specific config files with the parameters of the robot and the end effector used.

**NOTE** This package couldn't be tested on simulation, since the physics from Gazebo doens't allow to run the robot on effort.

## Documentation

In order to generate the documentation in doxygen execute:
```
$ roscd gravity_compensation_controller
$ rosdoc_lite .
```

This will generate **doc** folder where all the documentation is stored.