# NSL-2206AA ROS2
--- NSL-2206AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4

 ```
$ sudo vi /etc/udev/rules.d/defined_lidar.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777",SYMLINK+="ttyLidar"

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
![Screenshot from 2023-07-17 14-40-06](https://github.com/nano-roboscan/NSL-2206AA-ROS2/assets/106071093/9f048a4f-2750-4410-adcc-f7c7a7c1f21c)

![Screenshot from 2023-07-17 14-39-31](https://github.com/nano-roboscan/NSL-2206AA-ROS2/assets/106071093/aa9568cc-a2ae-482d-9b01-79f4bfa5c3af)

# Set parameters
```
$ rqt
 (reconfigure)
```
![Screenshot from 2023-07-17 14-54-13](https://github.com/nano-roboscan/NSL-2206AA-ROS2/assets/106071093/d4fbd0a6-1490-4804-8328-781712b872c1)



```
cvShow : Image Viewer on/off


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

 



 
 
 
