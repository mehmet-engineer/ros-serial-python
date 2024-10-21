# ros-serial-python
Example serial port communication python node in ROS1

**Mehmet Kahraman / 21.10.2024**

Main Requirements:
--
- Ubuntu 20.04 Focal
- ROS 1 Noetic Desktop Full

Installation
---
```
pip3 install pyserial
```

Run the Serial Node
---
```
rosrun force_sensor_serial sensor_serial.py
```
> Force sensor publisher started.
>
> Serial port opened. Port: /dev/ttyUSB0 Baud: 9600

> Force 2.0 N
> 
> ...

Call Calibration ROS Service
---
```
rosservice call /calibrate_force_sensor "{}"
```
> Calibrating force sensor...
