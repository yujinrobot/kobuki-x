
Half-ring of IR sensors
  - base to ring center static tf
  - IR readings to laserscan node

 To interface with waiterbot's Arduino Mega 2560 board, flash the firmware provided on Bosch drivers:

  $ roscd arduino_interface
  $ cd arduino_firmware
  $ export BOARD=mega2560
  $ export ARDUINO_DIR=/opt/arduino  # (or whatever you installed arduino software)
  $ make upload

 You must install the Arduino udev rule to make the board available on the default port "/dev/arduino":

  $ roscd waiterbot_sensors
  $ sudo cp resources/ir_scan/58-arduino.rules /etc/udev/rules.d
 
 and then replug the USB cable.
