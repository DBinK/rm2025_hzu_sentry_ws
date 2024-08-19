#!/bin/bash
echo  'KERNEL=="ttyUSB0", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar0"' >/etc/udev/rules.d/ydlidar.rules

echo  'KERNEL=="ttyACM0", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar0"' >/etc/udev/rules.d/ydlidar-V2.rules

echo  'KERNEL=="ttyUSB0", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar0"' >/etc/udev/rules.d/ydlidar-2303.rules

echo  'KERNEL=="ttyUSB1", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar1"' >/etc/udev/rules.d/ydlidar.rules

echo  'KERNEL=="ttyACM1", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar1"' >/etc/udev/rules.d/ydlidar-V2.rules

echo  'KERNEL=="ttyUSB1", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar1"' >/etc/udev/rules.d/ydlidar-2303.rules



service udev reload
sleep 1
service udev restart


