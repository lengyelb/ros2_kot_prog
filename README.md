# ros2_kot_prog

## About

Half year project for ROS2 subject. 

This is a project that controlls a dvrk robot to draw shapes id 3D space using interactive markers

## Usage

How to *build* and use the package.

    colcon build --symlink-install

Launch the dvrk main console

    ros2 run dvrk_robot dvrk_console_json -j ~/dvrk2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json

Launch the ROS 2 joint and robot state publishers

    ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1

Launch RViz

    ros2 run rviz2 rviz2 -d ~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz

*Optional* Lauinch rqt gui

    ros2 run rqt_gui rqt_gui

After launching RViz, you need to turn on and home the robot using the dvrk main console

![Screenshot 2024-04-24 103045](https://github.com/lengyelb/ros2_kot_prog/assets/92853275/dc60487c-9845-4093-b728-acfece248687)

Launch to projects launch file

    ros2 launch ros2_kot_prog kot_prog_launch.py

While the project is running, you need to add the approriate views to RViz

![Screenshot 2024-04-24 103525](https://github.com/lengyelb/ros2_kot_prog/assets/92853275/8b669018-7f9b-4b1d-9137-33978a27e227)

## Launch Parameters

- **v**: The speed used by the robot during TCP movements. Default value: `0.005`.

- **dt**: The time interval or loop rate. Default value: `0.01`.

- **omega**: The angular speed to use during JAW movements. Default value: `0.1`.

- **tcp_offset**: The offset between the tool center point and where the jaw grabs. Default value: `0.008`.

- **grab_error**: The maximum allowed error between TCP and different objects to consider them grabbed. Default value: `0.002`.

- **debounce_timeout**: The debounce timeout between two marker requests. Default value: `2`.

 - **shape**: The desired shape to be drawn using the markers. Default value: `'X'`. Allowed options are: 'X', '+', '-'

Example usage with all options

    ros2 launch ros2_kot_prog kot_prog_launch.py v:=0.005 dt:=0.01 omega:=0.1 tcp_offset:=0.008 grab_error:=0.002 shape:=X debounce_timeout:=2

Recommended example usage

    ros2 launch ros2_kot_prog kot_prog_launch.py shape:=X
