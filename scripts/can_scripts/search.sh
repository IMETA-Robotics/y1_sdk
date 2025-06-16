#!/bin/bash

udevadm info -a -n /dev/ttyACM* | grep serial