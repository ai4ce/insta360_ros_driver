#!/bin/bash
#For WSL, uncomment if unnecessary
sudo usbip attach -r 127.0.0.1 -b 4-1
#Replace with the camera port
sudo chmod 666 /dev/bus/usb/001/005