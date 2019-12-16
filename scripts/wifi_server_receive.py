#!/usr/bin/env python

try:
    import selectors
except ImportError:
    import selectors2 as selectors
import rospy
import socket
import json
from tircgo_msgs.msg import WifiIO
from tircgo_msgs.srv import Ask_Data,Send_Task
from std_msgs.msg import Header
from rospy_message_converter import message_converter
from rospy_message_converter import json_message_converter
from tircgo_msgs.msg import ControllerTalk

if rospy.has_param('Working_Station'):
	WORKING_STATION = rospy.get_param('Working_Station')
else:
	WORKING_STATION = False

# WORKING_STATION = True # True or False

def ros_serv_(p):  # call for service # input is string
        while(1):
                try:
                        rospy.wait_for_service('robot_wifi_askdata_inner',timeout=1) # wait until service available # service name
                        break
                except:
                        rospy.loginfo("Service call failed: task confirm")
	try:
		rospy.loginfo('SERVER:ask central for data')
		ask_data = rospy.ServiceProxy('robot_wifi_askdata_inner', Ask_Data) # handler; name, service name the --- one
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		purpose = p
		s = ask_data(header,purpose)
		rospy.loginfo('SERVER:ask central for data done')
		return s.ctrlData
	except rospy.ServiceException, e:
		rospy.loginfo("SERVER:Service call failed")

def send_wifi(dt):  # call for service
	rospy.wait_for_service('send_task') # wait until service available # service name
	try:
		rospy.loginfo('SERVER:send_wifi start')
		send_t = rospy.ServiceProxy('send_task', Send_Task) # handler; name, service name the --- one
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		s = send_t(header,dt)
		rospy.loginfo('SERVER:send_wifi done')
		# rospy.loginfo('seveice get :',s)
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("SERVER:Service call failed")

class Wifi():
	def __init__(self):
		rospy.init_node('wifi_server_receive', anonymous=True)
		self.pub = rospy.Publisher('robot_wifi_io', WifiIO , queue_size=30) # node, msg, size
		self.pub2 = rospy.Publisher('robot_wifi_controller_talk_inner', ControllerTalk , queue_size=10) # node, msg, size

		# self ID = ID , port number
		if rospy.has_param('IP_address'):
			self.ID = rospy.get_param('IP_address')
		else:
			self.ID = '127.0.0.1'
		if rospy.has_param('port'):
			self.port = rospy.get_param('port')
		else:
			self.port     = 12345
		self.hop_count= 5

	        # list of other car
                if rospy.has_param('host_list'):
                        h_list = rospy.get_param('host_list')
                        h_list = h_list.split(',')
                        self.host_list = []
                        for i in range(len(h_list)/2):
                                self.host_list.append((h_list[i*2],int(h_list[i*2+1])))

                else:
                        self.host_list       = [("192.168.1.101", 12346),("192.168.1.101",12345),("192.168.1.102",12345)]

		for i in range(len(self.host_list)):
        		if self.host_list[i][0] == self.ID and self.host_list[i][1] == self.port:
                		self.host_list.remove((self.ID ,self.port ))
				break
        	self.host_list = tuple(self.host_list)
		self.length_h_l = len(self.host_list)

		# data_init
		self.d = DATA()
		self.parameter()

		# initial node
		rospy.loginfo("SERVER:ID:"+str(self.ID)+" port:"+str(self.port))

		# wifi connection
		self.lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.lsock.bind((self.ID, self.port))
		self.lsock.listen(5) # listen to accept # listen to 5
		self.lsock.setblocking(False)
		self.sel = selectors.DefaultSelector()
		self.sel.register(self.lsock, selectors.EVENT_READ, data=None)# key
		rospy.loginfo('SERVER:listening on '+str (self.ID)+" with port "+str( self.port))

		#rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			events = self.sel.select(timeout=-1)#None
			for key, mask in events:# return key and event(), key is named tuple got smth     inside
				if key.data is None:
					self.accept_wrapper(key.fileobj)
				else:
					self.service_connection(key, mask)

	def parameter(self):
		self.TASK = 'W'
		self.OK   = 'O'
		self.COST = 'C'
		self.NODE = 'N'
		self.NODE_REPLY = 'A'
		self.JUST_TALK = 'J'
		self.WEB = 'B'
		self.sending_no = 1
		self.previous_sender = self.ID

	def accept_wrapper(self,sock): #its from listening socket and accept connection
		conn, addr = sock.accept()  # Should be ready to read
		rospy.loginfo('SERVER:accepted connection from'+str( addr))
		conn.setblocking(False)
		data = [addr]
		events = selectors.EVENT_READ | selectors.EVENT_WRITE
		self.sel.register(conn, events, data=data)

	def service_connection(self,key, mask):
		sock = key.fileobj
		data = key.data
		flag = 0
		if mask & selectors.EVENT_READ:
			recv_data = sock.recv(1024)  # Should be ready to read
			if recv_data:
				rospy.loginfo('SERVER:recv from : '+str(data[0]))
				flag,data_ = self.receive_data(recv_data) #
				self.previous_sender = data_.sender

		if mask & selectors.EVENT_WRITE: #normally write
			if flag != 0:
				if self.previous_sender == data and self.previous_sender!= self.ID: 
					rospy.loginfo('SERVER:rdy to reply data')
					self.reply_data(data_,flag) # maybe can use sock as signature to return
					flag = 0

	def receive_data(self,data_rc):
		rospy.loginfo('SERVER:start receive data')
		##################################################################
		self.d = json_message_converter.convert_json_to_ros_message('tircgo_msgs/WifiIO', data_rc.decode('utf-8'))
		##################################################################
		flag = 0
		for i in range(len(self.d.signatures)):
			self.d.signatures[i] =	eval(self.d.signatures[i])
		rospy.loginfo('SERVER:start receive data'+str(self.d.signatures))

		if self.d.purpose == self.TASK: # reply to working station
            		flag = 1
			self.d.signatures = [self.d.sender]
		elif self.d.purpose == self.NODE: # reply node
			flag = 2
			self.d.signatures = [self.d.sender]
		elif self.d.purpose == self.NODE_REPLY: # just rossend
			flag = 0
		elif self.d.purpose == self.COST: # remove signature then rossend
			flag = 0
		elif self.d.purpose == self.OK: # reply node ok
			flag = 0
		elif self.d.purpose == self.JUST_TALK: # reply node ok
			flag = 0
			self.rossend2(self.d.c_talk)
		        return flag,self.d

		if self.d.purpose == self.WEB :
			if WORKING_STATION == False : # recv WEB 
	        		rospy.loginfo('SERVER:done receive data web no need to pass to master')
	        		return flag,self.d
			else:
				flag = 0

		# means i truely receive this message so i sign
		for i in self.d.signatures:
			if self.ID == i[0]:
				self.d.signatures.remove(i)
				break

                if len(self.d.signatures) >= 1 :
                        rospy.loginfo('>1 : '+str(len(self.d.signatures)))
                        for i in range(len(self.d.signatures)):
                                #rospy.loginfo('sign : '+str(data.signatures[i]))
                                #rospy.loginfo('sign : '+str(type(str(data.signat$
                                self.d.signatures[i] = str(self.d.signatures[i])
                else:
                        self.d.signatures = []


		# send back~
		self.rossend(self.d)

	        rospy.loginfo('SERVER:done receive data ')
	        return flag,self.d


	def reply_data(self,data_rp,flag):
		if flag == 1:
			self.reply_ok(data_rp)
		elif flag == 2:
			self.reply_node(data_rp)
		#elif flag == 3:
		#	self.reply_cost(data_rp)

	def reply_ok(self,d):
		rospy.loginfo('SERVER:reply OK')
		d.purpose = 'O'
		send_wifi(d)

	def reply_node(self,d):
		rospy.loginfo("SERVER:start reply node req ")
		rospy.loginfo("SERVER:ask for node ")
		if WORKING_STATION :
	                if rospy.has_param('station_node'):
	                        r_n = rospy.get_param('station_node')
	                        r_n = r_n.split(',')
	                        route_ = int(r_n[0])
	                        node_  = int(r_n[1])
	                else:
	                        route_ = 0
	                        node_  = 0
			b = Ask_Data() # ask data reply
			b.np_ocp.route = route_
			b.np_ocp.node = node_
			b.np_ocp.pos.x = -1
			b.np_ocp.pos.y = -1
			b.np_ocp.pos.z = -1
		else:
			b = ros_serv_(d.purpose)
		rospy.loginfo('SERVER:reply for node service')
		d.purpose = 'A'
		d.node = b.nd_ocp
		send_wifi(d)
		rospy.loginfo('SERVER:end reply node req')

	def rossend(self,data):
		rospy.loginfo("SERVER:ros send data")
		self.pub.publish(data)

	def rossend2(self,data):
		rospy.loginfo("SERVER:ros send data just talk")
		self.pub2.publish(data)


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

if __name__ == '__main__':
    try:
        w = Wifi()
    except rospy.ROSInterruptException:
        rospy.loginfo('SERVER:error')
