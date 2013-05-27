#!/bin/bash

#sleep 30
nice -n -16 rosrun rosserial_python serial_node.py /dev/arduino
