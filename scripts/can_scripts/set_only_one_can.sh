#!/bin/bash
workspace=$(pwd)

if [ -e /dev/ttyACM* ]; then
  echo "Found ttyACM device"
else
  echo "No ttyACM device found"
  exit 1
fi

idVendor=$(udevadm info -a -n /dev/ttyACM* | grep idVendor | awk 'NR==1 {print substr($0, 23,4)}')
idProduct=$(udevadm info -a -n /dev/ttyACM* | grep idProduct | awk 'NR==1 {print substr($0, 24,4)}')
serial_number=$(udevadm info -a -n /dev/ttyACM* | grep serial | awk 'NR==1 {print substr($0, 21,12)}')

echo -e "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", ATTRS{serial}==\"$serial_number\", SYMLINK+=\"imeta_y1_can0\"" > imeta_y1_can.rules

sudo cp imeta_y1_can.rules /etc/udev/rules.d/

sudo chmod +x /etc/udev/rules.d/imeta_y1_can.rules

sudo udevadm control --reload-rules && sudo udevadm trigger

echo "/dev/imeta_y1_can0 symlink created!"