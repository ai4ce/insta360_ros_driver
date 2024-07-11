#!/bin/bash

#For WSL, uncomment if unnecessary
# sudo usbip attach -r 127.0.0.1 -b 4-1

#Add permissions for USB port
#TODO: Change to venderID = 0x2e1a
echo SUBSYSTEM=='"usb"', ATTR{product}=='"Insta360 ONE"', SYMLINK+='"insta"' | sudo tee /etc/udev/rules.d/99-insta.rules

#Trigger SYMLINK creation
sudo udevadm trigger

#Grant permission for camera port
sudo chmod 777 /dev/insta