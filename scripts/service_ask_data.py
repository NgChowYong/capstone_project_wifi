#!/usr/bin/env python

try:
    import selectors
except ImportError:
    import selectors2 as selectors
import rospy
import socket
import types
import json
import time as ti

from ros_wifi.msg import CtrlData
from std_msgs.msg import Header
from ros_wifi.srv import Ask_Data,Ask_DataResponse

def ad(req):
	# input is header and purpose string
	# output is ctrldata
	rospy.loginfo( 'recv purpose: ')
	rospy.loginfo( req.purpose  )
	
	data = Ask_DataResponse()
	data.ctrlData.mode = 3
	data.ctrlData.nd_ocp.route = 1
	data.ctrlData.nd_ocp.node = 1
	data.ctrlData.nd_ocp.pos.x = 1
	data.ctrlData.nd_ocp.pos.y = 1
	data.ctrlData.nd_ocp.pos.z = 1

	return data

def main():
    rospy.init_node('test_service', anonymous = True)
    rospy.Service('robot_wifi_askdata_inner', Ask_Data, ad)
    rospy.loginfo( 'service used to give ctrl data (node) : ')
    rospy.spin()

if __name__ == '__main__':
    main()

