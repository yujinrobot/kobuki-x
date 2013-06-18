#!/bin/bash

sleep 40
nice -n -16 rosrun rosserial_python serial_node.py /dev/arduino
