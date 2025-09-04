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
$ git clone --recurse-submodules https://github.com/nano-roboscan/NSL-2206AA-ROS2.git
$ cd NSL-2206AA-ROS2/NSL2206_driver
$ colcon build --packages-select roboscan_nsl2206
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 run roboscan_nsl2206 roboscan_publish_node
$ ros2 launch roboscan_nsl2206 camera.Launch.py

```

# NSL-2206AA View
<img width="328" height="178" alt="Image" src="https://github.com/user-attachments/assets/164554e8-cbea-4fa3-93c0-419a71e8fec5" />
<img width="868" height="703" alt="Image" src="https://github.com/user-attachments/assets/4e14e996-a96c-454e-97cf-cf37c407a299" />

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

 



 
 
 
