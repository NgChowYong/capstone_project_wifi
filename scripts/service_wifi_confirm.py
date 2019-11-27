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
from ros_wifi.srv import WifiTaskConfirm,WifiTaskConfirmResponse

def ad(req):
	# Cost task
	# ---
	# string error_code
	# bool is_taken
    rospy.loginfo( "input")
    rospy.loginfo( req.task)
    send_b = WifiTaskConfirmResponse()
    send_b.error_code = 'O'
    send_b.is_taken = 1

    return send_b

def main():
    rospy.init_node('service_wifo_confirm', anonymous = True)
    rospy.Service('robot_wifi_taskconfirm_inner', WifiTaskConfirm, ad)
    rospy.spin()

if __name__ == '__main__':
    main()

