#!/usr/bin/env python

try:
    import selectors
except ImportError:
    import selectors2 as selectors
import rospy
import socket
import json
from ros_wifi.msg import WifiIO
from ros_wifi.srv import Ask_Data,Send_Task
from std_msgs.msg import Header
from rospy_message_converter import message_converter
from rospy_message_converter import json_message_converter

if rospy.has_param('Working_Station'):
	WORKING_STATION = rospy.get_param('Working_Station')
else:
	WORKING_STATION = False

# WORKING_STATION = True # True or False

def ros_serv_(p):  # call for service # input is string 
	rospy.wait_for_service('robot_wifi_askdata_inner') # wait until service available # service name
	try:
		rospy.loginfo('ask central for data')
		ask_data = rospy.ServiceProxy('robot_wifi_askdata_inner', Ask_Data) # handler; name, service name the --- one
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		purpose = p
		s = ask_data(header,purpose)
		rospy.loginfo('ask central for data done')
		return s.ctrlData
	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed")

def send_wifi(dt):  # call for service
	rospy.wait_for_service('send_task') # wait until service available # service name
	try:
		rospy.loginfo('send_wifi start')
		send_t = rospy.ServiceProxy('send_task', Send_Task) # handler; name, service name the --- one
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		s = send_t(header,dt)
		rospy.loginfo('send_wifi done')
		# rospy.loginfo('seveice get :',s)
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed")

class Wifi():
	def __init__(self,addr="127.0.0.1"):
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

		# data_init
		self.d = DATA()
		self.parameter()

		# initial node
		rospy.init_node('wifi_server_receive', anonymous=True)
		self.pub = rospy.Publisher('robot_wifi_io', WifiIO , queue_size=30) # node, msg, size
		rospy.loginfo("ID:"+str(self.ID)+" port:"+str(self.port))

		# wifi connection
		self.lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.lsock.bind((self.ID, self.port))
		self.lsock.listen(5) # listen to accept # listen to 5
		self.lsock.setblocking(False)
		self.sel = selectors.DefaultSelector()
		self.sel.register(self.lsock, selectors.EVENT_READ, data=None)# key
		rospy.loginfo('listening on '+str (self.ID)+" with port "+str( self.port))

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
		self.WEB = 'B'
		self.sending_no = 1
		self.previous_sender = self.ID

	def accept_wrapper(self,sock): #its from listening socket and accept connection
		conn, addr = sock.accept()  # Should be ready to read
		rospy.loginfo('accepted connection from'+str( addr))
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
				rospy.loginfo('recv from : '+str(key.fileobj))
				flag,data_ = self.receive_data(recv_data.decode('utf-8')) #
				self.previous_sender = data_.sender

		if mask & selectors.EVENT_WRITE: #normally write
			if flag != 0:
				if self.previous_sender == data and self.previous_sender!= self.ID: 
					rospy.loginfo('rdy to reply data')
					self.reply_data(data_,flag) # maybe can use sock as signature to return
					flag = 0       

	def receive_data(self,data_rc):
		rospy.loginfo('start receive data')
		d = json.loads(data_rc)
		self.d = json_message_converter.convert_json_to_ros_message('ros_wifi/WifiIO', d)
		flag = 0

		if self.d.purpose == self.TASK: # reply to working station
            		flag = 1
			self.d.signatures = self.d.sender
		elif self.d.purpose == self.NODE: # reply node
			flag = 2
			self.d.signatures = self.d.sender
		elif self.d.purpose == self.NODE_REPLY: # just rossend
			flag = 0
		elif self.d.purpose == self.COST: # remove signature then rossend
			flag = 0
		elif self.d.purpose == self.OK: # reply node ok
			flag = 0
		
		if self.d.purpose == self.WEB and WORKING_STATION == False : # recv WEB 
	        	rospy.loginfo('done receive data ')
	        	return flag,self.d
		
		# means i truely receive this message so i sign 
		if self.ID in self.d.signatures:
			self.d.signatures.remove(self.ID)

		# send back~
		self.rossend(self.d)

	        rospy.loginfo('done receive data ')
	        return flag,self.d


	def reply_data(self,data_rp,flag):
		if flag == 1:
			self.reply_ok(data_rp)
		elif flag == 2:
			self.reply_node(data_rp)
		#elif flag == 3:
		#	self.reply_cost(data_rp)

	def reply_ok(self,d):
		rospy.loginfo('reply OK')
		d.purpose = 'O'
		send_wifi(d)

	def reply_node(self,d):
		rospy.loginfo("start reply node req ")
		rospy.loginfo("ask for node ")
		if WORKING_STATION :
			b = Ask_Data() # ask data reply
			b.np_ocp.route = -1
			b.np_ocp.node = -1
			b.np_ocp.pos.x = -1
			b.np_ocp.pos.y = -1
			b.np_ocp.pos.z = -1	
		else:
			b = ros_serv_(d.purpose) 
		rospy.loginfo('reply for node service')
		d.purpose = 'A'
		d.node = b.nd_ocp
		send_wifi(d)
		rospy.loginfo('end reply node req')

	def rossend(self,data): 
		rospy.loginfo("ros send data")
		self.pub.publish(data)

        
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
        rospy.loginfo('error')
