#!/usr/bin/env python
#AUTHOR: Younghun Ju <yhju@yujinrobot.comm>, <yhju83@gmail.com>

import argparse
import roslib; roslib.load_manifest('kobuki_node')
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from kobuki_msgs.msg import SensorState

class RangeReader(object):

	def __init__(self):
		rospy.init_node("getIrRange", anonymous=True)
		self.sub_core  = rospy.Subscriber("mobile_base/sensors/core", SensorState, self.SensorsCallback)
		self.pub_range = rospy.Publisher("range", Float64)


	def SensorsCallback(self,data):
		
		v = data.analog_input[int(self.args.port)]
		
		if self.args.sensor == '0':
			r = 2.97280807345002e-20*pow(v, 6) - 3.26552032146424e-16*pow(v, 5) + 1.44401467814260e-12*pow(v, 4) \
			  - 3.29557069570137e-09*pow(v, 3) + 4.13874167948414e-06*pow(v, 2) - 0.00282691862449452*v + 0.965998806885667
		elif self.args.sensor == '1':
			r = 5.77450227562403e-20*pow(v, 6) - 6.92745555808126e-16*pow(v, 5) + 3.30751853854928e-12*pow(v, 4) \
			  - 8.04696654429061e-09*pow(v, 3) + 1.06386602839130e-05*pow(v, 2) - 0.00754287882922655*v + 2.56001821404260
		else:
			print "Invalid sensor type: " + self.args.sensor
			return
		
		print "range: " + str(r) + " m"
		
		range = Float64()
		range = r
		self.pub_range.publish(range)

if __name__ == '__main__':
	try:
		instance = RangeReader()

		parser = argparse.ArgumentParser(description = 'It prints readings from one of the analog ports as a range.')
		parser.add_argument('-p', '--port', help = 'Analog port to read from (0..3)', required = True)
		parser.add_argument('-s', '--sensor', help = 'Connected sensor model (0: GP2D120, 1: GP2Y0A21YK)', required = True)
		instance.args = parser.parse_args()

		print 
		print "It prints readings from one of the analog ports as a range."
		print
		print ("Input port: A%s" % instance.args.port )
		print ("Sensor type: %s" % instance.args.sensor )

		rospy.spin()
	except rospy.ROSInterruptException: pass
