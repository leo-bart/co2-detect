#!/usr/bin/env python

''' Defines a ROS node that reads a Float32 from 
the co2_concentration topic and writes a Boolean to the 
gofango topic.
'''

import rospy
from std_msgs.msg import Float32, Bool

limitConcentration = 700.
allowedConcentration = 500.

# instantiates fan signal publisher
pub = rospy.Publisher('turn_fan_on', Bool, queue_size=10)

def checkconcentration(data):
	# first, writes value of concentration to log
    rospy.loginfo('CO2 concentration is {0:4.2f} ppm'.format(data.data))
    
    # now, checks if fan needs to be turned on
    if data.data >= limitConcentration:
    	rospy.loginfo('Turning fan on.')
    	pub.publish(Bool(1))
    elif data.data < allowedConcentration:
        rospy.loginfo('Turning fan off.')
        pub.publish(Bool(0))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('co2_concentration', Float32, checkconcentration)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
