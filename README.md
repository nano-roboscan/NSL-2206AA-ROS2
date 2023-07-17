# NSL-2206AA ROS2
--- NSL-2206AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4
 
 
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

# Set parameters
```
$ rqt
 (reconfigure)
```


cvShow : Image Viewer on/off
channel : 0 ~ 15

hdr_mode 0 : HDR off
hdr_mode 1 : Spatial HDR
hdr_mode 2 : Temperal HDR

imageType 0 : Grayscale 
imageType 1 : Distance 
imageType 2 : Distance / Amplitude
imageType 4 : Distance / Grayscale

integrationTime0 = 0 ~ 1000(VCSEL)

integrationTime0 = 0 ~ 10000

modIndex 0 : 10MHz
modIndex 1 : 20MHz


transformAngle : angle (rviz-based y-axis rotation)
```

 



 
 
 
