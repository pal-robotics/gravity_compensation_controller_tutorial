\mainpage .

# Gravity Compensation Controller Tutorial

This package shows the user how to implement a gravity compensation controller by using the ros_control interface.

In this package the user could learn how to:

- Implement an effort controller that overcomes gravity.
- Create his own controller with the ros_control API.

## Running on the robot

First install or deploy this package on your robot.

Then use the controller manager to stop all the active controllers that share resources with the gravity compensation controller.

In order to check which controllers are active, and which resources are using run:
```
$ rosservice call /controller_manager/list_controllers

``` 
This shows all the loaded controllers, and describes which ones are active and the resources claimed by every controller. 

Then it will be necessary to stop all those active controllers that use the same resources as the gravity compensation controller. For example, let's stop arm_controller.
```
$ rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'arm_controller'
strictness: 0" 

```
Finally load and start the gravity_compensation_controller_tutorial
```
$ roslaunch gravity_compensation_controller_tutorial gravity_compensation_controller_tutorial.launch robot:=robot_used end_effector:=end_effector_used
```
This launch file contains three arguments:

- **robot** : By default tiago. Is the robot where we launch this controller.
- **end_effector** : By default is pal-gripper. End effector mounted on the robot. There are three options: pal-gripper, pal-hey4, schunk-wsg.
- **simulation** : By default is false. If true, allows to run it on simulation.
- **spawn** : By default is true. If false, only loads the controller without starting it.
- **controller_ns** : By default is gravity_compensation_tutorial. Is the namespace of the controller and the controller name.

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

**NOTE:** This tutorial is supposed to be launched on the [TIAGo Robot](http://wiki.ros.org/Robots/TIAGo) from **PAL Robotics**. If the user has his own gripper or wants to extrapolate it to another robot, it will be necessary to regenerate the specific config files with the parameters of the robot and the end effector used.

## Simulation

There are some issues simulating the gravity compensation in Gazebo, due to how the physics are simulated. It doesn't allow it to test properly and with the default parameters.

Although, if the user wants to see it running on simulation, it will be necessar to put:
```
- motor_torque_constant: 1.0
- reduction_ratio: 1.0
```
for all actuated joints.

There are specific config files to launch the simulation with TIAGo. It will be necessary to follow the same steps as in the previous section, with the exception that in the launch file has to be specified simulation:=true
```
$ roslaunch gravity_compensation_controller_tutorial gravity_compensation_controller_tutorial.launch simulation:=true robot:=robot_used end_effector:=end_effector_used
```

## Learning by demonstration

Visit our repository [learning_gui](https://github.com/pal-robotics/learning_gui) to discover how to use this controller to teach new motions to the robot.

## Documentation

In order to generate the documentation in doxygen execute:
```
$ roscd gravity_compensation_controller
$ rosdoc_lite .
```

This will generate a **doc** folder inside the package where all the documentation is stored.