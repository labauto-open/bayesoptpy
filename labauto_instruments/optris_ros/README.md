# optris_ros
- ROS interface for optris thermography via serial communication
- optris_drivers is used for obtaining thermal image
- optrispy is used for obtaining temperature at the spesified area through the RS485 converter.

# Installation & setup
## Installation for optris_driver
- Install SDK deb
https://documentation.evocortex.com/libirimager2/html/Installation.html
- build optris_driver
  ```
  cd ~/catkin_ws/src
  git clone git@github.com:evocortex/optris_drivers.git
  cd optris_drivers
  catkin bt
  ```

## Hardware connection
plug both of thermography camera and RS485 converter to your computer.


# Sample program
```
roslaunch optris_ros xi80.launch
```
- you can subscribe
  - temperature of the main area configured via PIX Connect (Windows software)
  - thermal image
- You need unplug/plug USB connector when you stop/restart the launch file to avoid the following error.
```
Error: UVC device with serial 0 could not be found
```


# Links
- optris_driver
  - https://wiki.ros.org/optris_drivers
  - https://github.com/evocortex/optris_drivers/tree/master
