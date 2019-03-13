\mainpage .

# Gravity Compensation Controller Tutorial

This package shows the user how to implement a gravity compensation controller by using the ros_control interface.

In this package the user could learn how to:

- Implement an effort controller that overcomes gravity.
- Create his own controller with the ros_control API.

## Usage

First install or deploy the specific package on your robot.

Then use the controller manager to stop all the active controllers that share resources with this controller.

Finally run:
```
$ roslaunch gravity_compensation_controller_tutorial gravity_compensation_controller_tutorial.launch robot:=robot_used end_effector:=end_effector_used
```

This launch file contains two arguments:

- **robot** : By default tiago. Is the robot where we launch this controller.
- **end_effector** : By default is pal-gripper. End effector mounted on the robot. There are three options: pal-gripper, pal-hey4, schunk-wsg.

**NOTE:** This tutorial is supposed to be launched on the TIAGo Robot from **PAL Robotics**. If the user has his own gripper or wants to extrapolate it to another robot, it will be necessary to regenerate the specific config files with the parameters of the robot and the end effector used.

**NOTE** This package couldn't be tested on simulation, since the physics from Gazebo doens't allow to run the robot on effort.

## Documentation

In order to generate the documentation in doxygen execute:
```
$ roscd gravity_compensation_controller
$ rosdoc_lite .
```

This will generate **doc** folder where all the documentation is stored.