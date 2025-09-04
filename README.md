# NSL-2206AA ROS2
--- NSL-2206AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4

 ```
$ sudo vi /etc/udev/rules.d/defined_lidar.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="ttyNsl2206

$ service udev reload
$ service udev restart
 ```

2. Build NSL-2206AA demo
```
$ cd NSL2206_driver
$ colcon build --packages-select roboscan_nsl2206
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 run roboscan_nsl2206 roboscan_publish_node
$ ros2 launch roboscan_nsl2206 camera.Launch.py

```

# NSL-2206AA View


# Set parameters
```
$ rqt
 (reconfigure)
```



```
cvShow : Image Viewer on/off

imageType 1 : Distance 

imageType 2 : Distance / Amplitude

integrationTime0 = 0 ~ 1000(VCSEL)

integrationTime0 = 0 ~ 2000

modIndex 0 : 10MHz

modIndex 1 : 20MHz

transformAngle : angle (rviz-based y-axis rotation)
```

 



 
 
 
