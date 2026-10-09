#!/bin/bash

#For WSL, uncomment if unnecessary
# sudo usbip attach -r 127.0.0.1 -b 4-1

#Add permissions for USB port
# udevadm info --name /dev/bus/usb/003/026 --attribute-walk
echo SUBSYSTEM=='"usb"', ATTR{idVendor}=='"2e1a"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
#Reload and trigger udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

#Grant permission for camera port
# Wait for the symlink to be created
for i in {1..5}; do
    if [ -e /dev/insta ]; then
        sudo chmod 777 /dev/insta
        echo "Successfully set permissions for /dev/insta"
        break
    else
        echo "Waiting for /dev/insta to be created... (attempt $i/5)"
        sleep 1
    fi
done

if [ ! -e /dev/insta ]; then
    echo "Error: /dev/insta was not created. Please check if the camera is connected and in ANDROID mode."
    exit 1
fi
