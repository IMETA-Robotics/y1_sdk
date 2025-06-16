#!/bin/bash
workspace=$(pwd)

serial_number=$(udevadm info -a -n /dev/ttyACM* | grep serial | awk 'NR==1 {print substr($0, 21,12)}')
if [ -z "$serial_number" ]; then
    echo "No ttyACM* serial number found!"
    exit 1
fi

echo -e "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"16d0\", ATTRS{idProduct}==\"117e\", ATTRS{serial}==\"$serial_number\", SYMLINK+=\"imeta_y1_can0\"" > imeta_y1_can.rules

sudo cp imeta_y1_can.rules /etc/udev/rules.d/

sudo chmod +x /etc/udev/rules.d/imeta_y1_can.rules

sudo udevadm control --reload-rules && sudo udevadm trigger

echo "/dev/imeta_y1_can0 symlink created!"