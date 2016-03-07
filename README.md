# Eddie Robot

This is the eddie robot project that we have developed for the second section of our Programming Things module.

To run this project, you will need to clone this project and put it in a ROS Jade project. Once you have done that and used catkin to build it, you will need to run each of the commands below in a seperate terminal.

```
roscore 
rosrun eddie_serial_driver eddie_remote_publisher.py
rosrun eddie_serial_driver serial_driver.py
rosrun eddie_serial_driver shape_camera.py
```

There is a wiki available at [https://github.com/Swapnull/EddieRobot/wiki](https://github.com/Swapnull/EddieRobot/wiki)