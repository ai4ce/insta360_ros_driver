#!/bin/bash

#Add permissions for USB port
echo SUBSYSTEM=='"usb"', ATTR{product}=='"Insta360 ONE"', SYMLINK+='"insta"' | sudo tee /etc/udev/rules.d/99-insta.rules

#Trigger SYMLINK creation
sudo udevadm trigger

#Grant permission for camera port
sudo chmod 777 /dev/insta