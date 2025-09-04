#!/bin/bash

echo ""
echo "This script copies ROBOSCAN NSL-2206AA udev rules to /etc/udev/rules.d/"
echo ""

echo "NSL2206_driver (USB) : /dev/ttyACMx to /dev/ttyLiDAR :"

if [ -f "/etc/udev/rules.d/99-roboscan_nsl2206.rules" ]; then
	echo "99-roboscan_nsl2206.rules file already exist."
else
	echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777",SYMLINK+="ttyLiDAR"' > /etc/udev/rules.d/99-roboscan_nsl2206.rules
	echo '99-roboscan_nsl2206.rules created'
fi

echo ""
echo "Reload rules"
echo ""

#sudo udevadm control --reload-rules
#sudo udevadm trigger
sudo service udev reload
sudo service udev restart
	
#sudo vi /etc/udev/rules.d/defined_lidar.rules
#KERNEL=="ttyACM*", ATTRS{serial}=="205D386B5643", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777",SYMLINK+="ttyLidar"

# service udev reload
# service udev restart
