#!/bin/bash

sudo cp imeta_y1_can.rules /etc/udev/rules.d/

sudo chmod +x /etc/udev/rules.d/imeta_y1_can.rules

sudo udevadm control --reload-rules && sudo udevadm trigger

echo "/dev/imeta_y1_can0 symlink created!"