#!/usr/bin/env python
# this is used to act as master
# ideal is to create a service to shengming
# help processing ask node, ask cost part

import rospy
import threading
import time
from std_msgs.msg import Header
from ros_wifi.msg import WifiIO,RouteNode
from ros_wifi.srv import Ask_Data,Ask_DataResponse
from ros_wifi.srv import Send_Task,Send_TaskResponse
from ros_wifi.srv import WifiNodeOcp,WifiNodeOcpResponse
from ros_wifi.srv import WifiTaskConfirm,WifiTaskConfirmResponse
from ros_wifi.srv import WifiNodeCost,WifiNodeCostResponse


if rospy.has_param('Working_Station'):
	WORKING_STATION = rospy.get_param('Working_Station')
else:
	WORKING_STATION = False

# WORKING_STATION = True # True or False


def ros_serv_(p):  # call for service # input is string 
	rospy.wait_for_service('robot_wifi_askdata_inner') # wait until service available # service name
	try:
		# rospy.loginfo('ask central for data')
		ask_data = rospy.ServiceProxy('robot_wifi_askdata_inner', Ask_Data) # handler; name, service name the --- one
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		purpose = p
		s = ask_data(header,purpose)
		# rospy.loginfo('ask central for data done')
		return s.ctrlData
	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed:")


def ask_cost(rn):  # call for service # input is string 
	rospy.wait_for_service('robot_wifi_nodecost_inner') # wait until service available
	try:
		rospy.loginfo('ask central for cost')
		ask_cost = rospy.ServiceProxy('robot_wifi_nodecost_inner', WifiNodeCost) 
		s = ask_cost(rn)
		rospy.loginfo('ask central for cost done')
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed:")


def Task_confirm(cs):  # call for service # input is cost 
	rospy.wait_for_service('robot_wifi_taskconfirm_inner') # wait until service available
	try:
		rospy.loginfo('ask central for wifi confirm')
		task_c = rospy.ServiceProxy('robot_wifi_taskconfirm_inner', WifiTaskConfirm) 
		s = task_c(cs)
		rospy.loginfo('ask central for wifi confirm done')
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed:")


def send_wifi(data):
	rospy.wait_for_service('send_task')
	try:
		rospy.loginfo('tell wifi to send data')
		send_tt = rospy.ServiceProxy('send_task', Send_Task)
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		sss = Send_Task()
		sss.header = header
		sss.info = data
		rospy.loginfo(str(sss))
		ret = send_tt(sss)
		rospy.loginfo('tell wifi to send done')
		return ret
	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed:")



class WIFI_MASTER():
	def __init__(self):
		#setting up parameter
		self.parameter_setup()

		#create node and service for shengming use
		rospy.init_node('wifi_master', anonymous=True)
		s = rospy.Service('robot_wifi_nodeocp_outer', WifiNodeOcp, self.reply_service) # provide service 

		###
		starting = rospy.Service('starting_task',Send_Task,self.start_task)
		###

		rospy.Subscriber('robot_wifi_io',WifiIO,self.subs)

		rospy.loginfo("working : "+str(WORKING_STATION))
		rospy.loginfo('initialize wifi master service ')
		add_thread = threading.Thread(target = self.update_job)
  	  	add_thread.start()
		
		# for closing thread and node		
		rospy.spin()
		self.shut = 1

	def start_task(self,rep):
		self.button_press = 1
		rsp = Send_TaskResponse()
		return rsp

	def parameter_setup(self):
		# for shuttingdown node and thread 
		self.shut = 0		
		
		# database
		self.database = []
		if rospy.has_param('IP_address'):
			self.ID = rospy.get_param('IP_address')
		else:
			self.ID = '127.0.0.1'


		# some parameter
		self.COST = 'C'
		self.TASK = 'W'
		self.NODE = 'A'
		self.NODE_ASK = 'N'
		self.ERROR = 'O'
		self.current_task = 0 # just for reminding all task executing is under current task
		self.WEB  = 'B'

		# locking for update job and subs
		self.LOCK   = 1
		self.UNLOCK = 0		
		self.lock_1 = self.LOCK

		# list of other car
		self.host_list       = (("192.168.1.100", 12345),("192.168.1.101", 12345),("192.168.1.102", 12345))

		self.length_h_l = len(self.host_list)

		# some state from vehicle
		self.current_node = RouteNode()
		self.current_state = "IDLE" # got IDLE , COST_ING , COST_DONE , WORKING
		self.car_state = "IDLE" # got IDLE , HOMING , TEACHING , WORKING
		self.node_data = NODE_DATA(self.host_list)

		# button press
		self.button_press = 0

		if WORKING_STATION == True:
			self.WORKING_TAG = 1 
			
	def reply_service(self,req): # provide service reply
		# error code 0 for no error ; P for still processing
		rospy.loginfo('receive: ',req)
		rsp = WifiNodeOcpResponse()
		if current_state == "WORKING":
			if self.node_data.check_hit(self.current_node):
				rsp.is_ocp = 1
				rsp.error_code = '0'
				return rsp
			else:
				rsp.is_ocp = 0
				rsp.error_code = '0'
				return rsp
		else:		
			rsp.is_ocp = 0
			rsp.error_code = 'P'
			return 

	def subs(self,data): # read data from wifi # data is WifiIO
		# read new task
		rospy.loginfo('recv data from wifi with purpose : ',data.purpose)
		if data.purpose == self.TASK:
			check_flag = 0
			if not WORKING_STATION: # working station not need to recv and store TASK
				for i in range(len(self.database)): # check if this is old task
					if data.TASK_ID == self.database[i].task_id:
						check_flag = 1
						break
				if check_flag == 0: # append if not old task
					self.database.append(TASK_DATA(data))

		# read cost 
		elif data.purpose == self.COST: # there are some problem here
			if data.sender_state == "IDLE": # sender is normal ppl
				if len(self.database) > 0: # i got some task working
					flag_task = 0
					if data.TASK_ID == self.database[i].task_id:
						flag_task = 1
					if flag_task == 1: # task is within my task
						if self.current_state == "WORKING":
							self.reply_normal(data,'W') # reply without me only
						else:			
							if len(data.signatures) == 0: # i receive the last data
								if data.cost.cost_owner == self.ID: # if it is me then i m the chosen one
									# here do task confirmation
									s = Task_confirm(data.cost)
									self.current_state = "COST_DONE"
									if s.is_taken: # check if want to take
										self.current_state = "WORKING"
										self.current_task = data.TASK_ID
									else: 
										self.current_state = "IDLE"
								else: # receive the last but not me
									pass # then do nothing
							else: # not me receive the last
								# then just continue to pass with compare to my cost
								self.reply_compare(data,self.database)
					else: # task is not in my task
						self.reply_normal(data)
				else: # dont even have a task
					# return im normal
					self.reply_normal(data)
			elif data.sender_state == "WORKING":
				if self.current_state == "WORKING":
					self.reply_normal(data,'W')
				else:
					self.reply_normal(data)

		# read node 
		elif data.purpose == self.NODE:
			if not WORKING_STATION: # working station not need to recv and store TASK
				for i in range(len(self.database)): # check if this is old task # remove node task
					if data.TASK_ID == self.database[i].task_id:
						self.database.remove(self.database[i])
						break
				# renew node list 
				self.node_data.update(data)
				# if gonna hit will need to wait Shengming to ask !!!
			else:
				# asking working station their node current not write anything to prevent it is empty by default
				pass
		else:
			pass

		#########################################################################################
		# for working station used
		if WORKING_STATION :
			if data.purpose == self.WEB:
			# check if data to working station recv ?
			# 	do a function to adjust the thing 
				self.update_web(data.ctrldata)
		##########################################################################################

		if self.lock_1 == self.LOCK:
			while(1):
				pass
				if self.lock_1 == self.UNLOCK:
					break 
		self.lock_1 = self.LOCK
	
	def update_job(self):
		rospy.loginfo('start background process data')
		last = time.time()
		last_node = time.time()
		while(1):
			# reading all input and update to database with period as below
			if self.lock_1 == self.LOCK:
				self.lock_1 = self.UNLOCK

			current = time.time()	
			if current - last > 0.1 : # in sec
				last = current
				car = ros_serv_("N")
				self.current_node = car.nd_ocp
				# check for input and do process to state
				# got IDLE , HOMING , TEACHING , WORKING

				if car.mode & 1 :
					self.car_state = "IDLE" 
					# checking for task and calculate cost and save and send
					# check for exists of task database
					if len(self.database) == 0:
						pass
					else:
						self.start_ask_cost()

				elif car.mode & 4 :
					self.car_state = "HOMING"
				elif car.mode & 8 :
					self.car_state = "TEACHING"
					continue
				elif car.mode & 16 :
					self.car_state = "WORKING"
					# here start to keep asking node !!!
					current_node = time.time()	
					if current_node - last_node > 0.1 : # in sec
						last_node = current_node
						self.send_all_node()

			if self.shut == 1: # for closing this thread
				break
#########################################################################################
			# for working station used
			if WORKING_STATION :
				# check if button pressed
				# if pressed
				#	announce new task and send to all others
				# else:
				#	pass
				if self.button_pressed():
					rospy.loginfo('UI button pressed announce data')
					self.announce_new_task()	
				pass
			else: # if not working station need to send data constantly to WEB 
			########################################################################################
				pass
		rospy.loginfo('done')

##############################################################################
	def button_pressed(self):
		if self.button_press == 0:
			return False
		else:
			rospy.loginfo("button pressed")
			self.button_press = 0
			return True

	def announce_new_task(self):# sending task , task node , author
		w = WifiIO()
		w.signatures = ["ALL"]
		w.TASK_ID = str(time.time())
		w.node = self.current_node
		w.author = self.ID
		s = send_wifi(w) # ask other cost

	def update_web(self,ctrldata):# function provided by Meng
		return 1
##############################################################################

	def sent_all_node(self):
		rospy.loginfo('send all')
		w = WifiIO()
		w.signatures = ["ALL"]
		w.TASK_ID = self.current_task
		w.node = self.current_node
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		s = send_wifi(Send_Task(header,w)) # ask other cost


	def reply_normal(self,data,state="I"): # input is wifiIO
		rospy.loginfo('reply normal')
		w = data
		w.signatures = data.signatures
		w.cost  = data.cost
		w.sender_state = state
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		s = send_wifi(Send_Task(header,w)) # ask other cost

	def reply_compare(self,data,task): # input is wifiIO and database
		rospy.loginfo('reply compare')
		w = data
		for task_ in task:
			if task_.task_id == data.TASK_ID:
				if data.cost.cost >  task_.self_cost: # this means my cost is lower
					w.cost.cost_owner = self.ID
					w.cost			  = task_.self_cost
		w.sender_state = "I"
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		s = send_wifi(Send_Task(header,w)) # ask other cost

	def start_ask_cost(self):
		rospy.loginfo('start ask cost')
		self.current_task = self.database.pop(0)
		ret = ask_cost(task.task_node) # calc self cost
		self.current_task.self_cost = ret.cost
		# NOW ask others their cost 
		# need to send task need to use WIFI IO now
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		w = self.wifiio_cost_update(self.current_task)
		w.sender_state = "I"
		s = send_wifi(Send_Task(header,w)) # ask other cost
		self.current_state = "COST_ING"


	def wifiio_cost_update(self,data): # input is current task, a task obj
		rospy.loginfo('cost update')
		w = WifiIO()
		w.author = self.ID
		# sender is done in client part
		w.TASK_ID = data.task_id
		w.signatures = "ALL"
		w.cost.cost    = data.self_cost
		w.cost.cost_owner = self.ID
		w.cost.target  = data.task_node
		return w

class TASK_DATA():
	def __init__(self, data): # inpput is wifi IO
		self.task_id = data.TASK_ID
		self.task_node = data.node
		self.task_sender = data.author
		self.self_cost = -1
		self.lowest_cost = -1


class NODE_DATA():
	def __init__(self,data_): # input data should be host list
		self.data = []	
		self.length = len(data_)
		for i in range(self.length):
			self.data.append((data_[i][0],RouteNode())) # host ID , routenode
	
	def check_hit(self,my_node): # will return will hit or not # check for next node
		for i in range(self.length):
			if my_node.route == self.data[i][1].route and my_node.node == self.data[i][1].node: 
				return True # will hit 
		return False # will not hit

	def update_data(self,data_): # input is wifiio
		for i in range(self.length):
			if self.data[i][0] == data_.sender:
				self.data[i][1] = data_.node
				break


if __name__ == '__main__':
    #try:
    w = WIFI_MASTER()
	#  w.talker()
    #except rospy.ROSInterruptException:
    #    rospy.loginfo('error')
    #    pass
