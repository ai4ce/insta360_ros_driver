#!/bin/bash

#For WSL, uncomment if unnecessary
# sudo usbip attach -r 127.0.0.1 -b 4-1

#Add permissions for USB port
# udevadm info --name /dev/bus/usb/003/026 --attribute-walk
echo SUBSYSTEM=='"usb"', ATTR{idVendor}=='"2e1a"', SYMLINK+='"insta"' | sudo tee /etc/udev/rules.d/99-insta.rules

#Trigger SYMLINK creation
sudo udevadm trigger

#Grant permission for camera port
sudo chmod 777 /dev/insta