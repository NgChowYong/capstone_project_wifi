#!/usr/bin/env python

try:
    import selectors
except ImportError:
    import selectors2 as selectors
import rospy
import socket
import types
import json
import copy
from ros_wifi.msg import WifiIO,Cost,RouteNode
from std_msgs.msg import Header,String
from ros_wifi.srv import Send_Task,Send_TaskResponse
from rospy_message_converter import json_message_converter
from ros_wifi.msg import ControllerTalk

if rospy.has_param('Working_Station'):
	WORKING_STATION = rospy.get_param('Working_Station')
else:
	WORKING_STATION = False

# WORKING_STATION = True # True or False

class Wifi():
	def __init__(self):
		rospy.init_node('wifi_client_send')
		rospy.Subscriber("robot_wifi_controller_talk_outer", ControllerTalk, self.just_talk)
		self.ser = rospy.Service('send_task', Send_Task, self.handle_wifi_send)  # could need to add another class reference to direct the pointer to the class 

		if rospy.has_param('IP_address'):
			self.ID = rospy.get_param('IP_address')
		else:
			self.ID = '127.0.0.1'

		if rospy.has_param('port'):
			self.port = rospy.get_param('port')
		else:
			self.port     = 12345

		self.hop_count= 5


		if rospy.has_param('host_list'):
                        h_list = rospy.get_param('host_list')
                        h_list = h_list.split(',')
                        self.host_list = []
                        for i in range(len(h_list)/2):
                                self.host_list.append((h_list[i*2],int(h_list[i*2+1])))

                else:
                        self.host_list       = [("192.168.1.101", 12346),("192.168.1.101",12345),("192.168.1.102",12345)]$
	        for i in range(len(self.host_list)):
        		if self.host_list[i][0] == self.ID and self.host_list[i][1] == self.port:
                		self.host_list.remove((self.ID ,self.port ))
				break

        	self.host_list = tuple(self.host_list)
		# data_init
		self.d = DATA()
		self.parameter()

		# init node and provide service
		rospy.loginfo("current host list : "+str(self.host_list))
		# server_addr = (host, port)
		self.server_addr = list(self.host_list)
		self.s_no = 0
		# start to connect to multiple host
		self.sel = selectors.DefaultSelector()
		for i in range(len(self.server_addr)):
			connid = i + 1
 			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			sock.setblocking(False)
			sock.connect_ex(self.host_list[i]) # == 0 : # 0 for success
			# self.s_no = self.s_no + 1 # not everytime will connect
			events = selectors.EVENT_READ | selectors.EVENT_WRITE
			data = (self.host_list[i])
			self.sel.register(sock, events, data=data)
			# sock.connect_ex(self.host_list[i]) # == 0 : # 0 for success
		#self.sending_no = s_no
	        self.sending_no = len(self.host_list)
		rospy.loginfo('we connec robot no : '+str(self.sending_no))
		rospy.loginfo('we actual connec robot no : '+str(self.s_no))
		rospy.loginfo('connection done wait for service call : ')
		rospy.spin()

	def parameter(self):
		self.TASK = 'W'
		self.OK   = 'O'
		self.COST = 'C'
		self.NODE = 'N'
		self.WEB  = 'B'
		self.NODE_REPLY = 'A'
		self.JUST_TALK = 'J'
		self.sending_no = 1

	def wifi_send(self,sock,data__):
		data = copy.deepcopy(data__)
		if len(data.signatures) > 0 :
			rospy.loginfo('>1 : '+str(len(data.signatures)))
			for i in range(len(data.signatures)):
				#rospy.loginfo('sign : '+str(data.signatures[i]))
				#rospy.loginfo('sign : '+str(type(str(data.signatures[i]))))
				data.signatures[i] = str(data.signatures[i])
		#else:
		#	data.signatures = [str(data.signatures)]

		rospy.loginfo('start wifi send'+str(data.signatures))
		# json.dumps(data)
		j = json_message_converter.convert_ros_message_to_json(data)
		j = j.encode('utf-8')
		sock.sendall(j)
		rospy.loginfo('sent data !! ')

	def accept_wrapper(self,sock): #its from listening socket and accept connection
		#conn, addr = sock.accept()  # Should be ready to read
		#rospy.loginfo('accepted connection from : '+str( addr))
		#conn.setblocking(False)
		#data = [addr,conn]
		#events = selectors.EVENT_READ | selectors.EVENT_WRITE
		#self.sel.register(conn, events, data=data)
		pass

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
				# rospy.loginfo('should not receive any data here !!!!')
				pass

		if mask & selectors.EVENT_WRITE:
			for i in self.d.signatures:
				if data[0] == i[0]:
					# could be improved by pruning of send signatures
					rospy.loginfo('start sending wifi ')
					# rospy.loginfo self.d
					try:
						self.wifi_send(sock,self.d)
					except:
						rospy.loginfo('error in sending data !!!!!!!!! ')
						return 2
					#	key.fileobj = key.fileobj.connect_ex(key.data)
					# rospy.loginfo self.d
					rospy.loginfo('sending to connection'+str(data))
					return 1
		return 0

	def just_talk(self,data):
		w = WifiIO()
		w.author = self.ID
		w.purpose = self.JUST_TALK
		w.signatures = ['ALL']
		w.c_talk = data
		rospy.wait_for_service('send_task')
		try:
			rospy.loginfo('CLIENT: tell wifi send just talk')
			send_tt = rospy.ServiceProxy('send_task',Send_Task)
			head = Header(stamp=rospy.Time.now(), frame_id='base')
			ret = send_tt(head,w)
			rospy.loginfo('CLIENT: wifi sending done')
		except rospy.ServiceException , e:
			rospy.loginfo('CLIENT: service call just talk')


	def handle_wifi_send(self,req): # input of service is  header and wifiio output is error code and header
		rospy.loginfo('start client')
		rospy.loginfo(str(req.info.signatures))
		send_no = self.sending_no
		# read input data
		head = req.header
		self.d = req.info
		self.d.sender = self.ID
		# note that author decide by master only

		# for clearing string tuple problem
		if self.d.signatures[0] != "ALL":
			while not isinstance(self.d.signatures[0],tuple):
				self.d.signatures = self.d.signatures[0]
			for i in range(len(self.d.signatures)):
        	                self.d.signatures[i] =  eval(self.d.signatures[i])

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

		elif req.info.purpose == self.JUST_TALK:
			send_no = 1
			if req.info.signatures[0] == "ALL": # all will send to every but else will send to original signatures ppl
				self.d.signatures = []
				# for i in range(len(self.host_list)):
				for i in range(send_no):
					self.d.signatures.append(self.host_list[i])

		elif req.info.purpose == self.OK:
			send_no = 1
			pass

		elif req.info.purpose == self.TASK or  req.info.purpose == self.WEB :
			#send_no = len(self.host_list)
			send_no = send_no
			if req.info.signatures[0] == "ALL": # all will send to every but else will send to original signatures ppl
				self.d.signatures = []
				# for i in range(len(self.host_list)):
				for i in range(send_no):
					self.d.signatures.append(self.host_list[i])

		self.server_addr = list(self.d.signatures)
		# currently msg sending is one to one only not one to many for every message, except for task 
		rospy.loginfo('client recv want to send data with purpose : '+str(req.info.purpose))
		rospy.loginfo('need to send to : '+str(self.d.signatures))
		flag = 0
		while not rospy.is_shutdown():
			#try:
			if True:
				events = self.sel.select(timeout=5)  #none
				for key, mask in events:   # return key and event(), key is named tuple got smth inside
					if key.data is None:
						#self.accept_wrapper(key.fileobj)
						#rospy.loginfo('accp ')
						# reconnect
						#key.fileobj.connect_ex(key.data)
						pass
				    	else:
						fff = 0
						if key.data in self.server_addr:
							fff = self.service_connection(key,mask)
						if fff == 2:
							fff = 0
							self.sel.unregister(key.fileobj)
				 			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
							sock.setblocking(False)
							sock.connect_ex(key.data) # == 0 : # 0 for success
							events = selectors.EVENT_READ | selectors.EVENT_WRITE
							data = key.data
							self.sel.register(sock, events, data=data)
						elif fff == 1:
							# rospy.loginfo(str(self.server_addr))
							self.server_addr.remove(key.data)
						flag += fff
				# for service return
				if len(self.server_addr) == 0:
				#if flag == send_no:
				    resp = Send_TaskResponse()
				    resp.header = Header(stamp=rospy.Time.now(), frame_id='base')
				    resp.error_code = '1'  # return int16 success # success~~
				    rospy.loginfo('client done and closing')
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
