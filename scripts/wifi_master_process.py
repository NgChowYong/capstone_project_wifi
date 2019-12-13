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
from flask import Flask,request,render_template

if rospy.has_param('Working_Station'):
	WORKING_STATION = rospy.get_param('Working_Station')
else:
	WORKING_STATION = False
# for share data of web app and code
counter = 0

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
		rospy.loginfo('MASTER:ask central for cost')
		ask_cost = rospy.ServiceProxy('robot_wifi_nodecost_inner', WifiNodeCost) 
		s = ask_cost(rn)
		rospy.loginfo('MASTER:ask central for cost done')
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("MASTER:Service call failed:")


def Task_confirm(cs):  # call for service # input is cost
	rospy.wait_for_service('robot_wifi_taskconfirm_inner') # wait until service available
	try:
		rospy.loginfo('MASTER:ask central for wifi confirm')
		task_c = rospy.ServiceProxy('robot_wifi_taskconfirm_inner', WifiTaskConfirm) 
		s = task_c(cs)
		rospy.loginfo('MASTER:ask central for wifi confirm done')
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("MASTER:Service call failed:")


def send_wifi(data):
	rospy.wait_for_service('send_task')
	try:
		rospy.loginfo('MASTER:tell wifi to send data')
		send_tt = rospy.ServiceProxy('send_task', Send_Task)
		header_h = Header(stamp=rospy.Time.now(), frame_id='base')
		ret = send_tt(header_h,data)
		rospy.loginfo('MASTER:tell wifi to send done')
		return ret
	except rospy.ServiceException, e:
		rospy.loginfo("MASTER:Service call failed:")

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
		rospy.loginfo("MASTER:working : "+str(WORKING_STATION))
		rospy.loginfo('MASTER:initialize wifi master service ')

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

		# get IP add and port
		if rospy.has_param('IP_address'):
			self.ID = rospy.get_param('IP_address')
		else:
			self.ID = '127.0.0.1'

	        if rospy.has_param('port'):
			self.port = rospy.get_param('port')
		else:
			self.port     = 12345

		# some parameter
		self.cc = 0
		self.COST = 'C'
		self.TASK = 'W'
		self.NODE = 'A'
		self.NODE_ASK = 'N'
		self.ERROR = 'O'
		self.current_task = 0 # just for reminding all task executing is under current task
		self.WEB  = 'B'
		self.IDLE = "IDLE"
		self.COSTING = "COST_ING"
		self.COSTDONE = "COST_DONE"
		self.WORKING = "WORKING"
		self.HOMING = "HOMING"
		self.TEACHING = "TEACHING"

		# locking for update job and subs
		self.LOCK   = 1
		self.UNLOCK = 0
		self.lock_1 = self.LOCK
		self.lock_once = 0

		# list of other car
		self.host_list       = [("192.168.1.101", 12346),("192.168.1.101", 12345),("192.168.1.102", 12345)]
	        for i in range(len(self.host_list)):
            		if self.host_list[i][0] == self.ID and self.host_list[i][1] == self.port:
                		self.host_list.remove((self.ID ,self.port ))
				break
        	self.host_list = tuple(self.host_list)

		self.length_h_l = len(self.host_list)

		# some state from vehicle
		self.current_node = RouteNode()
		self.current_state = self.IDLE # got IDLE , COST_ING , COST_DONE , WORKING
		self.current_state_flag = 0
		self.car_state = self.IDLE # got IDLE , HOMING , TEACHING , WORKING
		self.node_data = NODE_DATA(self.host_list)
		self.current_task = None
		self.task_tag = 0

		# database
		self.database = []

		# button press
		self.button_press = 0

		if WORKING_STATION == True:
			self.WORKING_TAG = 1

	def reply_service(self,req): # provide service reply
		# error code 0 for no error ; P for still processing
		rospy.loginfo('receive: '+str(req))
		rsp = WifiNodeOcpResponse()
		if current_state == self.WORKING:
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
			return rsp

	def subs(self,data): # read data from wifi # data is WifiIO
		# read new task
		rospy.loginfo('MASTER:recv data in master')
		while self.lock_once == 1: # must process each at once otherwise collision of data or mixing of data
			pass
		self.lock_once = 1
		#rospy.loginfo(str(data))
		rospy.loginfo('MASTER:recv data from wifi with purpose : '+str(data.purpose))
		if data.purpose == self.TASK:
			check_flag = 0
			if not WORKING_STATION: # working station not need to recv and store TASK
				for i in range(len(self.database)): # check if this is old task
					if data.TASK_ID == self.database[i].task_id:
						rospy.loginfo('MASTER:tasked aldy in database')
						check_flag = 1
						break
				if check_flag == 0: # append if not old task
					self.database.append(TASK_DATA(data))
					rospy.loginfo('MASTER:adding new task to database')

		# read cost
		elif data.purpose == self.COST: # there are some problem here
			rospy.loginfo('MASTER: recv in cost')
			#rospy.loginfo('MASTER: cost: '+str(data.cost))
			if data.sender_state == self.IDLE: # sender is normal ppl
				rospy.loginfo('MASTER:sender is idle')
				if len(self.database) > 0 or self.current_task != None : # i got some task working
					rospy.loginfo('MASTER: IM GOT SOME TASK')
					flag_task = 0
					if len(self.database)>0:
						for i in range(len(self.database)):
							if data.TASK_ID == self.database[i].task_id :
								flag_task = 1
								temp_flag = 1
								break
					elif self.current_task.task_id == data.TASK_ID:
						flag_task = 1
						temp_flag = 2
					if flag_task == 1: # task is within my task
						rospy.loginfo('MASTER: task is in my list')
						if self.current_state == self.WORKING:
							self.reply_normal(data,self.WORKING) # reply without me only
							rospy.loginfo('MASTER: IM WORKING')
						else:
							rospy.loginfo('MASTER: IM IDLE')
							if len(data.signatures) == 0: # i receive the last data
								if data.cost.cost_owner == self.ID and data.cost.cost >= self.current_task.self_cost  : # if it is me then i m the chosen one
									rospy.loginfo('MASTER: IM GOING to do the job')
									# here do task confirmation
									s = Task_confirm(data.cost)
									self.current_state = self.COSTDONE
									if s.is_taken: # check if want to take
										self.current_state = self.WORKING
										self.current_task = data.TASK_ID
										self.current_state_flag = 1
									else:
										self.current_state = self.IDLE
								else: # receive the last but not me
									self.current_task = None
									rospy.loginfo('MASTER: IM NOT lowest cost')
									pass # then do nothing
							else: # not me receive the last
								# then just continue to pass with compare to my cost
								rospy.loginfo('MASTER: Continue pass cost')
								if temp_flag == 1:
									self.reply_compare(data,self.database[i])
								if temp_flag == 2:
									self.reply_compare(data,self.current_task)
					else: # task is not in my task
						rospy.loginfo('MASTER:I didnt recv this task')
						self.reply_normal(data)
				else: # dont even have a task
					rospy.loginfo('MASTER:I didnt recv this task')
					# return im normal
					self.reply_normal(data)
			elif data.sender_state == self.WORKING:
				if self.current_state == self.WORKING:
					rospy.loginfo('MASTER:sender is WORKING im WORKING')
					self.reply_normal(data,self.WORKING)
				else:
					rospy.loginfo('MASTER:sender is WORKING im IDLE')
					self.reply_normal(data)

		# read node
		elif data.purpose == self.NODE:
			if not WORKING_STATION: # working station not need to recv and store TASK
				rospy.loginfo('MASTER:Node recv and updating')
				for i in range(len(self.database)): # check if this is old task # remove node task
					if data.TASK_ID == self.database[i].task_id:
						self.database.remove(self.database[i])
						break
				# renew node list
				self.node_data.update(data)
				# if gonna hit will need to wait Shengming to ask !!!
			else:
				rospy.loginfo('MASTER:im working station dont care node asking')
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
		self.lock_once = 0

	def update_job(self):
		rospy.loginfo('MASTER:start background process data')
		last = time.time()
		last2 = time.time()
		last_node = time.time()
		while(1):
			# reading all input and update to database with period as below
			if self.lock_1 == self.LOCK:
				self.lock_1 = self.UNLOCK

			current = time.time()
			if current - last > 0.49 : # in sec
				last = current
				try:
					car = ros_serv_("N")
				except:
					rospy.loginfo('MASTER:closed ros_serv')

				if current - last2 > 2:
					last2 = current
					self.send_update_web(car)

				self.current_node = car.nd_ocp
				# check for input and do process to state
				# got IDLE , HOMING , TEACHING , WORKING

				if car.mode & 1 :
					rospy.loginfo("MASTER: center is idle")
					self.car_state = self.IDLE
					if self.current_task != None and self.current_state_flag == 1:
						self.current_state_flag = 0
						self.current_task = None
						rospy.loginfo("MASTER: closing current task")
						pass
					# checking for task and calculate cost and save and send
					# check for exists of task database
					if len(self.database) == 0:
						pass
					else:
						rospy.loginfo('MASTER:processing task')
						self.start_ask_cost()

				elif car.mode & 4 :
					rospy.loginfo("MASTER: center is homing")
					self.car_state = self.HOMING
				elif car.mode & 8 :
					self.car_state = self.TEACHING
					rospy.loginfo("MASTER: center is teaching")
					continue
				elif car.mode & 16 :
					self.car_state = self.WORKING
					rospy.loginfo("MASTER: center is working")
					# here start to keep asking node !!!
					current_node = time.time()
					if current_node - last_node > 1 : # in sec
						last_node = current_node
						self.send_all_node()
					self.task_tag = 1

			if self.shut == 1: # for closing this thread
				break
			#########################################################################################
			# for working station used
			if WORKING_STATION :
				if self.button_pressed():
					rospy.loginfo('MASTER:UI button pressed announce data')
					self.announce_new_task()
				pass
			else: # if not working station need to send data constantly to WEB 
			########################################################################################
				pass
		rospy.loginfo('MASTER:all done')

	##############################################################################
	def button_pressed(self):
		if counter == 0 :
			return False
		else:
			self.cc = counter

		if self.button_press == self.cc:
			return False
		else:
			self.button_press = self.cc
			return True

	def announce_new_task(self):# sending task , task node , author
		w = WifiIO()
		w.purpose=self.TASK
		w.signatures = ["ALL"]
		w.TASK_ID = str(self.ID)+str(self.cc)
		w.node = self.current_node
		w.author = self.ID
		s = send_wifi(w) # ask other cost

	def update_web(self,ctrldata): # collect data and update web
		rospy.loginfo('MASTER: update web here')
		pass

	def send_update_web(self,ctrldata): # send data to web
		rospy.loginfo('MASTER: send update web')
		w = WifiIO()
		w.purpose=self.WEB
		w.signatures = ["ALL"]
		w.node = self.current_node
		w.author = self.ID
		w.ctrldata = ctrldata
		s = send_wifi(w) # ask other cost

	##############################################################################

	def sent_all_node(self):
		rospy.loginfo('MASTER:send all')
		w = WifiIO()
		w.signatures = ["ALL"]
		w.TASK_ID = self.current_task
		w.node = self.current_node
		header = Header(stamp=rospy.Time.now(), frame_id='base')
		w.purpose = self.NODE_ASK
		w.sender_state = self.IDLE
		w.author = self.ID
		s = send_wifi(w) # ask other cost


	def reply_normal(self,data,state="IDLE"): # input is wifiIO
		rospy.loginfo('MASTER:reply normal')
		w = data
		w.signatures = data.signatures
		w.cost  = data.cost
		w.sender_state = state
		# header = Header(stamp=rospy.Time.now(), frame_id='base')
		w.purpose = self.COST
		# w.sender_state = self.current_state
		s = send_wifi(w) # ask other cost

	def reply_compare(self,data,task): # input is wifiIO and database
		rospy.loginfo('MASTER:reply compare')
		w = data
		#for task_ in task:
		#if task_.task_id == data.TASK_ID:
		if data.cost.cost >  task.self_cost: # this means my cost is lower
			rospy.loginfo('MASTER:IM better')
			w.cost.cost	  = task.self_cost
			w.cost.cost_owner = self.ID
		w.purpose = self.COST
		w.sender_state = self.IDLE
		#header = Header(stamp=rospy.Time.now(), frame_id='base')
		s = send_wifi(w) # ask other cost

	def start_ask_cost(self):
		rospy.loginfo('MASTER:start ask cost')
		self.current_task = self.database.pop(0)
		rospy.loginfo('MASTER:calc cost')
		ret = ask_cost(self.current_task.task_node) # calc self cost
		rospy.loginfo('MASTER:get cost: '+str(ret.cost))
		self.current_task.self_cost = ret.cost
		# NOW ask others their cost
		# need to send task need to use WIFI IO now
		#header = Header(stamp=rospy.Time.now(), frame_id='base')
		w = self.wifiio_cost_update(self.current_task)
		w.sender_state = self.current_state
		w.purpose = self.COST
		s = send_wifi(w) # ask other cost
		self.current_state = self.COSTING


	def wifiio_cost_update(self,data): # input is current task, a task obj
		rospy.loginfo('MASTER:cost update')
		w = WifiIO()
		w.author = self.ID
		# sender is done in client part
		w.TASK_ID = data.task_id
		w.signatures = ["ALL"]
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

def apprun():
	app=Flask(__name__)

	@app.route('/',methods=['GET'])
	def initialize():
		global counter
		counter = 0
		return render_template('index.html',call=counter)

	@app.route('/submit',methods=['POST'])
	def submit():
		global counter
		counter += 1
		return render_template('index.html',call=counter)

	@app.route('/reset',methods=['POST'])
	def reset():
		global counter
		counter = 0
		return render_template('index.html',call=counter)

	app.run(host='0.0.0.0',debug=False,port=5000)


if __name__ == '__main__':
	# run web app
	if WORKING_STATION:
		add_thread = threading.Thread(target = apprun)
  		add_thread.start()
    #try:
    	w = WIFI_MASTER()
	#  w.talker()
    #except rospy.ROSInterruptException:
    #    rospy.loginfo('error')
    #    pass
