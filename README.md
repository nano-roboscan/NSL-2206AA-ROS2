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
$ colcon build
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 launch roboscan_nsl2206 camera.Launch.py
```

# NSL-2206AA View
<img width="1280" height="355" alt="Image" src="https://github.com/user-attachments/assets/66f76163-5e29-4a2b-8480-3c55dbcb1176" />

<img width="1026" height="928" alt="Image" src="https://github.com/user-attachments/assets/cb43e67b-80d8-45f3-80dc-fe580951443c" />

 



 
 
 
