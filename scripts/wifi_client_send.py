#!/usr/bin/env python

try:
    import selectors
except ImportError:
    import selectors2 as selectors
import rospy
import socket
import types
import json

from ros_wifi.msg import WifiIO,Cost,RouteNode
from std_msgs.msg import Header,String
from ros_wifi.srv import Send_Task,Send_TaskResponse
from rospy_message_converter import json_message_converter

if rospy.has_param('Working_Station'):
	WORKING_STATION = rospy.get_param('Working_Station')
else:
	WORKING_STATION = False

# WORKING_STATION = True # True or False

class Wifi():
	def __init__(self):
		if rospy.has_param('IP_address'):
			self.ID = rospy.get_param('IP_address')
		else:
			self.ID = '127.0.0.1'

		if rospy.has_param('port'):
			self.port = rospy.get_param('port')
		else:
			self.port     = 12345

		self.hop_count= 5

		# host_list = ID , port number
		#self.host_list(('127.0.0.1',12345),('127.0.0.1',12346))
		# current use 1 case first
		self.host_list       = [("192.168.1.101", 12346),("192.168.1.101", 12345),("192.168.1.102", 12345)]
	        for i in range(len(self.host_list)):
        		if self.host_list[i][0] == self.ID and self.host_list[i][1] == self.port:
                		self.host_list.remove((self.ID ,self.port ))
        	self.host_list = tuple(self.host_list)

		# data_init
		self.d = DATA()
		self.parameter()

		# init node and provide service
		rospy.init_node('wifi_client_send')
		self.ser = rospy.Service('send_task', Send_Task, self.handle_wifi_send)  # could need to add another class reference to direct the pointer to the class 

		# server_addr = (host, port)
		server_addr = []
		for i in range(len(self.host_list)):
			server_addr.append(self.host_list[i])
		s_no = 0
		# start to connect to multiple host
		self.sel = selectors.DefaultSelector()
		for i in range(len(server_addr)):
			connid = i + 1
 			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			sock.setblocking(False)
			#if sock.connect_ex(server_addr[i]) == 0: # 0 for success
			s_no = s_no + 1 # not everytime will connect
			events = selectors.EVENT_READ | selectors.EVENT_WRITE
			data = (connid,[self.d])
			self.sel.register(sock, events, data=data)
		#self.sending_no = s_no
	        self.sending_no = len(self.host_list)
		rospy.loginfo('we connec robot no : '+str(self.sending_no))
		rospy.loginfo('connection done wait for service call : ')
		rospy.spin()

	def parameter(self):
		self.TASK = 'W'
		self.OK   = 'O'
		self.COST = 'C'
		self.NODE = 'N'
		self.WEB  = 'B'
		self.NODE_REPLY = 'A'
		self.sending_no = 1

	def wifi_send(self,sock,data):
		rospy.loginfo('start wifi send')
		j = json_message_converter.convert_ros_message_to_json(data)
		# json.dumps(data)
		rospy.loginfo('json done')
		sock.sendall(j.encode('utf-8'))
		rospy.loginfo('sent data !! ')

	def accept_wrapper(self,sock): #its from listening socket and accept connection
		conn, addr = sock.accept()  # Should be ready to read
		rospy.loginfo('accepted connection from : '+str( addr))
		conn.setblocking(False)
		data = [addr,self.msg]
		events = selectors.EVENT_READ | selectors.EVENT_WRITE
		self.sel.register(conn, events, data=data)

	def service_connection(self,key, mask): # its client socket.
		sock = key.fileobj
		data = key.data

		if mask & selectors.EVENT_READ:
			#pass
			try:
				recv_data = sock.recv(1024)  # Should be ready to read
				if recv_data:
					rospy.loginfo('received from connection'+str( data[0]))
			except:
				rospy.loginfo('should not receive any data here !!!!')

		if mask & selectors.EVENT_WRITE:
			if data[0] in self.d.signatures:
				rospy.loginfo('start sending wifi ')
				# rospy.loginfo self.d
				self.wifi_send(sock,self.d)
				# rospy.loginfo self.d
				rospy.loginfo('sending to connection'+str(data[0]))
				return 1
		return 0

	def handle_wifi_send(self,req): # input of service is  header and wifiio output is error code and header

		send_no = self.sending_no
		# read input data
		head = req. header
		self.d = req.info
		self.d.sender = self.ID
		# note that author decide by master only

		if req.info.purpose == self.NODE_REPLY: # routenode details is renewed in master node 
			self.d.purpose = 'A'
			send_no = 1
			self.d.signatures = req.info.author # send back to author
		elif req.info.purpose == self.COST:
			send_no = 1
			self.d.purpose = 'C'
			if req.info.signatures[0] == "ALL": # all will send to every but else will send to original signatures ppl
				self.d.signatures = []
				# for i in range(len(self.host_list)):
				for i in range(send_no):
					self.d.signatures.append(self.host_list[i])

		elif req.info.purpose == self.NODE:
			send_no = 1
			pass
		elif req.info.purpose == self.OK:
			send_no = 1
			pass
		elif req.info.purpose == self.TASK or  req.info.purpose == self.WEB :
			#send_no = len(self.host_list)
			send_no = send_no

		rospy.loginfo('data storing done with purpose'+str(req.info.purpose))

		# currently msg sending is one to one only not one to many for every message, except for task 
		flag = 0
		while not rospy.is_shutdown():
			#try:
			if True:
				events = self.sel.select(timeout=-1)#None
				for key, mask in events:# return key and event(), key is named tuple got smth inside
					if key.data is None:
						self.accept_wrapper(key.fileobj)
				    	else:
						flag += self.service_connection(key,mask)
				# for service return
				if flag == send_no:
				    resp = Send_TaskResponse()
				    resp.header = Header(stamp=rospy.Time.now(), frame_id='base')
				    resp.error_code = '1'  # return int16 success # success~~
				    rospy.loginfo('done and closing')
				    flag = 0
				    return resp
			#except :
			else:
				resp = Send_TaskResponse()
				resp.header = Header(stamp=rospy.Time.now(), frame_id='base')
				resp.error_code = '0'  # return int16 success # fail @@
				rospy.loginfo('error')
				flag = 0
				return resp

class DATA():
	def __inti__(self):
		# data content in WifiIO
		self.data = WifiIO()
		self.data.purpose = 'O'
		self.data.sender = 'A'
		self.data.author = 'A'
		self.data.TASK_ID = 'A'
		self.data.error_code = '0'
		self.data.hop_count = 0
		self.data.signatures = 'O'
		self.data.cost = Cost()
		self.data.node = RouteNode()

if __name__ == "__main__":
    w = Wifi()
