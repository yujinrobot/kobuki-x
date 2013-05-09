#!/bin/bash

#sleep 40
#rosrun rosserial_python serial_node.py /dev/arduino

#sudo su -
#source /home/turtlebot/cafe_solution/devel/setup.bash
nice -n -11 rosrun rosserial_python serial_node.py /dev/arduino
#exit
#sudo su - turtlebot
# si de verdad funciona, hacerlo bien   @sudo ...  o  preguntar a daniel


##########   asi no va; error:  -su: no job control in this shell