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
from ros_wifi.srv import WifiNodeOcp,WifiNodeOcpResponse

def ad(req):
    rospy.loginfo('ask node input')
    rospy.loginfo(req.q_rn)
    send_b = Ask_DataResponse()
    send_b.error_code = 'O'
    send_b.is_ocp = 0

    return send_b

def main():
    rospy.init_node('service_ask_node', anonymous = True)
    rospy.Service('robot_wifi_nodeocp_inner', WifiNodeOcp, ad)
    rospy.spin()

if __name__ == '__main__':
    main()

