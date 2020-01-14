#!/usr/bin/env python
# this is used to act as master
# ideal is to create a service to shengming
# help processing ask node, ask cost part

import rospy
import threading
import time
from std_msgs.msg import Header
from tircgo_msgs.msg import WifiIO,RouteNode,CtrlData,Cost
from tircgo_msgs.srv import Ask_Data,Ask_DataResponse
from ros_wifi.srv import Send_Task,Send_TaskResponse
from tircgo_msgs.srv import WifiNodeOcp,WifiNodeOcpResponse
from tircgo_msgs.srv import WifiTaskConfirm,WifiTaskConfirmResponse
from tircgo_msgs.srv import WifiNodeCost,WifiNodeCostResponse
from flask import Flask,request,render_template


if rospy.has_param('Working_Station'):
	WORKING_STATION = rospy.get_param('Working_Station')
else:
	WORKING_STATION = False

#################################################################
# WEB_DATA is a class for storing data from website for station #
#################################################################

class WEB_DATA():
	def __init__(self):
		self.counter = 0 # used to remind click how many times and used to send task ID
		self.Robot_1 = ['Robot_1', 'IDLE', 'not started']
		self.Robot_2 = ['Robot_2', 'IDLE', 'not started']
		self.text = "STARTING"
		self.route_node = [0,0,0,0] # from 0,0 to 0,0
		self.reset_flag = 0

	def update_rn(self):
		# read input from web as x,y,a,b => from x,y to a,b
		no_ = self.text.split(',')
		# reject if input length not equal 4
		if len(no_) != 4 or len(no_)!=2:
			rospy.loginfo("WEB_INPUT ERROR 1")
			return 0
		# save data
		if len(no_)==2: # only given call station need to at least move back a bit
			try:
				self.route_node[0]=int(no_[0])
				self.route_node[1]=int(no_[1])
				self.route_node[2]=int(no_[0])
				self.route_node[3]=int(no_[1])
			except:
				rospy.loginfo("WEB_INPUT ERROR 2")
				return 0
		else:
			for i in range(4):
				try:
					self.route_node[i] = int(no_[i])
				except:
					rospy.loginfo("WEB_INPUT ERROR 3")
					return 0

	def set_counter(self,counter = 0):
		self.counter = counter

	def update_counter(self):
		self.counter += 1

	def get_c(self):
		return self.counter

# for share data of web app and code
# initial web_data when code begin, like initial a database
web_data = WEB_DATA()
web_data.set_counter()


###############################################
# below is a series of service initialization #
###############################################

def ros_node(nod):  # call for service # for ask node ocp
	# while loop pooling for keep asking node until central repeat
	while(1):
		try:
			rospy.wait_for_service('robot_wifi_nodeocp_inner',timeout=5) # wait until service available # service name
			break
		except:
			rospy.loginfo("Service call failed: ask node")
			if self.shut == 1: # for closing this thread
				break
	try:
		ask_node = rospy.ServiceProxy('robot_wifi_nodeocp_inner', WifiNodeOcp) # handler; name, service name the --- one
		s = ask_node(nod)
		return s.is_ocp # boolean is occupied
	except rospy.ServiceException, e:
		rospy.loginfo("Service call failed:")


def ros_serv_(p):  # call for service # for ask data(internal state)
	while(1):
		try:
			rospy.wait_for_service('robot_wifi_askdata_inner',timeout=5) # wait until service available # service name
			break
		except:
			rospy.loginfo("Service call failed: ask data")
			if self.shut == 1: # for closing this thread
				break

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


def ask_cost(rn):  # call for service # for ask cost
	while(1):
		try:
			rospy.wait_for_service('robot_wifi_nodecost_inner',timeout=5) # wait until service available # service name
			break
		except:
			if self.shut == 1: # for closing this thread
				break
			rospy.loginfo("Service call failed: ask cost")
	try:
		rospy.loginfo('MASTER:ask central for cost')
		ask_cost = rospy.ServiceProxy('robot_wifi_nodecost_inner', WifiNodeCost)
		s = ask_cost(rn)
		rospy.loginfo('MASTER:ask central for cost done')
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("MASTER:Service call failed:")


def Task_confirm(cs,dest):  # call for service # for task confirmation
	while(1):
		try:
			rospy.wait_for_service('robot_wifi_taskconfirm_inner',timeout=5) # wait until service available # service name
			break
		except:
			if self.shut == 1: # for closing this thread
				break
			rospy.loginfo("Service call failed: task confirm")
	try:
		rospy.loginfo('MASTER:ask central for wifi confirm')
		task_c = rospy.ServiceProxy('robot_wifi_taskconfirm_inner', WifiTaskConfirm)
		s = task_c(cs,dest)
		rospy.loginfo('MASTER:ask central for wifi confirm done')
		return s
	except rospy.ServiceException, e:
		rospy.loginfo("MASTER:Service call failed:")


def send_wifi(data): # ask client to send data through wifi
	rospy.wait_for_service('send_task')
	try:
		rospy.loginfo('MASTER:tell wifi to send data')
		if len(data.signatures)==0:
			rospy.loginfo("MASTER: empty send wifi")
			return 0
		send_tt = rospy.ServiceProxy('send_task', Send_Task)
		header_h = Header(stamp=rospy.Time.now(), frame_id='base')
		ret = send_tt(header_h,data)
		rospy.loginfo('MASTER:tell wifi to send done')
		return ret
	except rospy.ServiceException, e:
		rospy.loginfo("MASTER:Service call failed:")

################################################################
# WIFI_MASTER is the most important node                       #
# => decide sending and reply what kind of message             #
# => keep pooling internal state and website for update states #
################################################################

class WIFI_MASTER():
	def __init__(self):

		# create node and service for shengming use
		rospy.init_node('wifi_master', anonymous=True)

		# setting up parameter
		self.parameter_setup()
		# setting up service and topic kind of thing
		# provide service
		s = rospy.Service('robot_wifi_nodeocp_outer', WifiNodeOcp, self.reply_service)
		rospy.Subscriber('robot_wifi_io',WifiIO,self.subs)
		rospy.loginfo("MASTER: is working station? : "+str(WORKING_STATION))
		rospy.loginfo('MASTER:initialize wifi master service ')

		# start thread to keep pooling internal states
		add_thread = threading.Thread(target = self.update_job)
  	  	add_thread.start()

		rospy.spin()

		# for closing thread and node
		self.shut = 1

	def parameter_setup(self):
		# for shutting down node and thread
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
		self.COST = 'C'
		self.TASK = 'W'
		self.NODE_REPLY = 'A'
		self.NODE_ASK = 'N'
		self.WEB  = 'B'
		self.ERROR = 'O'

		# self state
		self.IDLE     = "IDLE"
		self.COSTING  = "COST_ING"
		self.COSTDONE = "COST_DONE"
		self.WORKING  = "WORKING"
		self.HOMING   = "HOMING"
		self.TEACHING = "TEACHING"

		# locking for update job and states # if not lock will induce memory conflict problem
		self.LOCK   = 1
		self.UNLOCK = 0
		self.lock_1 = self.LOCK
		self.lock_once = self.UNLOCK

		# prepare station and robot list
		self.station_list = []
		self.robot_list = []

		# list of other car
		if rospy.has_param('station_list'):
			h_list = rospy.get_param('station_list')
			h_list = h_list.split(',')

			for i in range(len(h_list)/2):
				# remove self from list
				if self.ID == h_list[i*2] and self.port == int(h_list[i*2+1]):
					pass
				else:
					# append into list
					self.station_list.append((h_list[i*2],int(h_list[i*2+1])))
		else:
			self.station_list       = [("192.168.1.102", 12345)]

		# list of other car
		if rospy.has_param('robot_list'):
			h_list = rospy.get_param('robot_list')
			h_list = h_list.split(',')

			# used to update web data # currently is static
			try:
				web_data.Robot_1[0] = h_list[0]
				web_data.Robot_1[1] = 'starting'
				web_data.Robot_1[2] = 0
			except:
				web_data.Robot_1[0] = 'starting'
				web_data.Robot_1[1] = 'starting'
				web_data.Robot_1[2] = 'starting'
			try:
				web_data.Robot_2[0] = h_list[2]
				web_data.Robot_2[1] = 'starting'
				web_data.Robot_2[2] = 0
			except:
				web_data.Robot_2[0] = 'starting'
				web_data.Robot_2[1] = 'starting'
				web_data.Robot_2[2] = 'starting'

			for i in range(len(h_list)/2):
				# remove self from list
				if self.ID == h_list[i*2] and self.port == int(h_list[i*2+1]):
					pass
				else:
					# append into list
					self.robot_list.append((h_list[i*2],int(h_list[i*2+1])))
		else:
			self.robot_list       = [("192.168.1.102", 12345)]

		self.station_list = tuple(self.station_list)
		self.length_st_list = len(self.station_list)
		self.robot_list = tuple(self.robot_list)
		self.length_rb_list = len(self.robot_list)

		# initial node
		if rospy.has_param('station_node'):
			r_n = rospy.get_param('station_node')
			r_n = r_n.split(',')
			route_ = int(r_n[0])
			node_  = int(r_n[1])
		else:
			route_ = 0
			node_  = 0

		# initial state of vehicle
		self.current_node = RouteNode()
		self.current_node.route = route_
		self.current_node.node = node_
		self.node_flag = 0
		self.node_flag2 = 1
		self.target_node = RouteNode()
		self.node_list = []
		for i in range(self.length_rb_list):
			self.node_list.append([self.robot_list[i][0],0]) # ip and ocp?
		self.node_count = 0

		# current task state
		self.current_state = self.IDLE # got IDLE , COST_ING , COST_DONE , WORKING
		self.current_state_flag = 0 # to avoid multi thread data mixing
		self.car_state = self.IDLE # got IDLE , HOMING , TEACHING , WORKING
		#self.node_data = NODE_DATA(self.robot_list)
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
		rospy.loginfo('want to go: '+str(req.q_rn.route)+'_'+str(req.q_rn.node))
		rsp = WifiNodeOcpResponse()

		# skip asking process direct return no car
		rsp.is_ocp = 0
		rsp.error_code = '0'
		return rsp

		self.node_flag = 1 # node_flag 1 for start asking node

		# flag for checking
		if self.node_flag2 == 0: # asking done
			rsp.is_ocp = 1
			rsp.error_code = '0'
			for i in range(self.length_rb_list):
				if self.node_list[i][1] == 1:
					rsp.is_ocp = 0
					rsp.error_code = '0'
					self.node_flag = 0
					break
			self.node_flag = 0
			self.node_flag2 = 1
			self.node_count = 0
			return rsp
		else: #processing
			self.node_flag2 = 1
			self.target_node = req.q_rn
			rsp.is_ocp = 0
			rsp.error_code = '0'
			return rsp

	##########################################################################
	# this fucntion will reply according to different case, a bit messy here #
	##########################################################################

	def subs(self,data): # read data from wifi # data is WifiIO

		# read new task
		rospy.loginfo('MASTER: recv data in master')
		# must wait process until free otherwise collision of data or mixing of data
		while self.lock_once == self.LOCK:
			pass
		self.lock_once = self.LOCK
		rospy.loginfo('MASTER: recv data from wifi with purpose : '+str(data.purpose))

		# recv data
		if data.purpose == self.TASK:
			if not WORKING_STATION: # working station not need to recv and store TASK
				check_flag = 0
				for i in range(len(self.database)): # check if this is old task
					if data.TASK_ID == self.database[i].task_id:
						rospy.loginfo('MASTER: tasked aldy in database')
						check_flag = 1

					if self.current_task != None:
						if data.TASK_ID == self.current_task.task_id:
							rospy.loginfo('MASTER: tasked aldy in my current task')
							check_flag = 1

				if check_flag == 0: # append if not old task
					self.database.append(TASK_DATA(data))
					rospy.loginfo('MASTER: adding new task to database')

		# read cost # if only one robot wont ask cost and wont reply cost
		elif data.purpose == self.COST: # there are some problem here
			rospy.loginfo('MASTER: recv in cost')
			# rospy.loginfo('MASTER: cost: '+str(data.cost))
			if data.sender_state != self.WORKING: # sender is normal ppl
				rospy.loginfo('MASTER:sender is idle')
				if len(self.database) > 0 or self.current_task != None or not WORKING_STATION: # i got some task working or working state
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
						elif self.current_state != self.WORKING:
							rospy.loginfo('MASTER: IM IDLE')
							if len(data.signatures) == 0: # i receive the last data
								if  data.cost.cost >= self.current_task.self_cost  : # if it is me then i m the chosen one
								#if data.cost.cost_owner == self.ID and data.cost.cost >= self.current_task.self_cost  : # if it is me then i m the chosen one
									rospy.loginfo('MASTER: IM GOING to do the job')
									dest = data.node
									# here do task confirmation
									s = Task_confirm(data.cost,dest)
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
						self.database.append(TASK_DATA(data))
						rospy.loginfo('MASTER: adding new task to database')
						rospy.loginfo('MASTER: calc cost')
						ret = ask_cost(data.cost.target) # reply float 
						rospy.loginfo('MASTER: get cost: '+str(ret.cost))
						data.cost.cost = ret
						data.cost.cost_owner = self.ID
						self.reply_normal(data)
				else: # dont even have a task
					rospy.loginfo('MASTER:I didnt recv this task')
					# return im normal
					self.reply_normal(data)
			else :  # data.sender_state == self.WORKING:
				if self.current_state == self.WORKING:
					rospy.loginfo('MASTER:sender is WORKING im WORKING')
					self.reply_normal(data,self.WORKING)
				else:
					rospy.loginfo('MASTER:sender is WORKING im IDLE')
					self.reply_normal(data)

		# read node
		elif data.purpose == self.NODE_REPLY:
			if not WORKING_STATION: # working station not need to recv and store TASK
				rospy.loginfo('MASTER:Node recv and updating')
				for i in range(len(self.database)): # check if this is old task # remove node task
					if data.TASK_ID == self.database[i].task_id:
						self.database.remove(self.database[i])
						break
				# renew node list
				for i in range(self.length_rb_list):
					if self.node_list[i][0] == data.author:
						self.node_list[i][1] = data.node.pos.x
						self.node_count += 1
				if self.node_count >= self.length_rb_list:
					self.node_count = 0
					self.node_flag2 = 0
					rospy.loginfo("MASTER: done asking all")
				else:
					self.node_flag2 = 1
				#self.node_data.update(data)
				# if gonna hit will need to wait Shengming to ask !!!
			else:
				rospy.loginfo('MASTER:im working station dont care node asking')
				# asking working station their node current not write anything to prevent it is empty by default
				pass
		else:
			pass

		# for working station used
		if WORKING_STATION :
			if data.purpose == self.WEB:
				self.update_web(data)

		# for locking problem again
		if self.lock_1 == self.LOCK:
			while(1):
				if self.lock_1 == self.UNLOCK:
					break
		self.lock_1 = self.LOCK
		self.lock_once = self.UNLOCK

	#########################################
	# this fucntion will keep updating data #
	#########################################

	def update_job(self):
		rospy.loginfo('MASTER:start background process data')
		last = time.time()
		last2 = time.time()
		last_node = time.time()
		while(1):
			# reading all input and update to database with period as below
			if self.lock_1 == self.LOCK:
				self.lock_1 = self.UNLOCK

			if not WORKING_STATION :
				current = time.time()
				if current - last > 1 : # in sec
					last = current
					try:
						# start asking node if asked by central
						if self.node_flag == 1:
							self.node_flag = 0 # ask once is enough "maybe"
							rospy.loginfo("MASTER: start ask node")
							self.send_all_node(self.target_node)
							rospy.loginfo("MASTER: start ask node done")
						else:
							rospy.loginfo("MASTER: not start @@ ask node")

						# ask for current state
						car = ros_serv_("N")
						self.current_node = RouteNode()
						if car.mode & 1 :
							rospy.loginfo("MASTER: center is idle")
							self.car_state = self.IDLE

							# data collision lock and delete current task lock
							if self.current_task != None and self.current_state_flag == 1:
								self.current_state_flag = 0
								self.current_task = None
								rospy.loginfo("MASTER: closing current task")
								pass

							# checking for task and calculate cost and save and send
							# check for exists of task database
							if len(self.database) == 0:
								pass
							else: # self exists task
								rospy.loginfo('MASTER:processing task')
								if self.length_rb_list == 0:
									rospy.loginfo('MASTER: IM the only one to do the job')

									# here do task confirmation
									# robot list is empty and got a task in database
									rospy.loginfo('MASTER: start ask cost')
									self.current_task = self.database.pop(0)
									rospy.loginfo('MASTER: calc cost')
									ret = ask_cost(self.current_task.task_node) # reply float 
									rospy.loginfo('MASTER: get cost: '+str(ret.cost))
									self.current_task.self_cost = ret.cost
									rospy.loginfo("Master: maybe got error")

									# generate cost
									c = Cost()
									c.cost_owner = self.current_task.task_sender
									c.cost 	     = ret.cost
									c.target     = self.current_task.task_node
									rospy.loginfo("MASTER: start task confirm")

									dest = self.current_task.final_node

									# send task confirm
									s = Task_confirm(c,dest) # input : cost msg
									self.current_state = self.COSTDONE
									if s.is_taken: # check if want to take
										self.current_state = self.WORKING
										self.current_task = data.TASK_ID
										self.current_state_flag = 1
									else:
										self.current_state = self.IDLE
								else:
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
							self.task_tag = 1

						# update website
						current_node = time.time()
						if current - last2 > 10:
							last2 = current
							self.send_update_web(car)
					except:
						rospy.loginfo('MASTER: not connected ros_serv or error')
			else:
			# for working station used
				if self.button_pressed():
					rospy.loginfo('MASTER:UI button pressed announce data')
					self.announce_new_task()

			if self.shut == 1: # for closing this thread
				break

		rospy.loginfo('MASTER:all done')

	#################################
	# some functions related to web #
	#################################

	def button_pressed(self):
		global web_data
		# check for button press
		if web_data.get_c() == 0: # initial counter is 0
			return False
		else:
			temp = web_data.get_c() # sometimes counter will keep jump to 0 and counter

		# if reset is pressed then reset button press
		if web_data.reset_flag == 1:
			web_data.reset_flag = 0
			self.button_press = 0
			return False

		if self.button_press == temp: # only true when renewed
			return False
		else:
			self.current_node.route = web_data.route_node[0]
			self.current_node.node = web_data.route_node[1]
			self.button_press = temp
			return True

	def announce_new_task(self):# sending task , task node , author
		w = WifiIO()
		w.purpose=self.TASK
		w.signatures = ["ALL"]
		w.TASK_ID = str(self.ID)+"-"+str(self.cc)
		w.node.route = web_data.route_node[2]
		w.node.node  = web_data.route_node[3]
		w.cost.target = self.current_node
		w.author = self.ID
		s = send_wifi(w) # ask other cost

	def update_web(self,data): # collect data and update web
		rospy.loginfo('MASTER: update web here')
                if data.author == web_data.Robot_1[0]:
                    web_data.Robot_1[1] = data.ctrldata.mode
		    web_data.Robot_1[2] = str(data.node.route)+'_'+str(data.node.node)
		if data.author == web_data.Robot_2[0]:
                    web_data.Robot_2[1] = data.ctrldata.mode
		    web_data.Robot_2[2] = str(data.node.route)+'_'+str(data.node.node)

	def send_update_web(self,ctrldata): # send data to web
		rospy.loginfo('MASTER: send update web')
		w = WifiIO()
		w.purpose=self.WEB
		w.signatures = ["ALL"]
		w.node = self.current_node
		w.author = self.ID
		w.ctrldata = ctrldata
		s = send_wifi(w) # ask other cost

	def send_all_node(self,node): #input is target node # ask node
		rospy.loginfo('MASTER: ask node_ing')
		w = WifiIO()
		w.signatures = ["ALL"]
		if self.current_task == None:
			w.TASK_ID = "NONE"
		else:
			w.TASK_ID = self.current_task
		w.node = node
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
		w.purpose = self.COST
		w.node = data.node
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
		w.node = data.node
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
		w.node = data.final_node
		return w

class TASK_DATA():
	def __init__(self, data): # inpput is wifi IO
		self.task_id = data.TASK_ID
		self.task_node = data.cost.target # or data.node
		self.task_sender = data.author
		self.self_cost = -1
		self.lowest_cost = -1
		self.final_node = data.node

# --------------------------------------------------------
#class NODE_DATA():
#	def __init__(self,data_): # input data should be host list
#		self.data = []
#		self.length = len(data_)
#		for i in range(self.length):
#			self.data.append((data_[i][0],RouteNode())) # host ID , routenode
#
#	def check_hit(self,my_node): # will return will hit or not # check for next node
#		for i in range(self.length):
#			if my_node.route == self.data[i][1].route and my_node.node == self.data[i][1].node:
#				return True # will hit
#		return False # will not hit
#
#	def update_data(self,data_): # input is wifiio
#		for i in range(self.length):
#			if self.data[i][0] == data_.author:
#				self.data[i][1] = data_.node
#				break
# -------------------------------------------------------

def apprun():
	app=Flask(__name__)
	global web_data

	@app.route("/")
	def index():
	    return render_template('index.html',call=web_data.get_c(),
	                                        Robot_1_Name=web_data.Robot_1[0],Robot_1_Status=web_data.Robot_1[1],Robot_1_Node=web_data.Robot_1[2],
	                                        Robot_2_Name=web_data.Robot_2[0],Robot_2_Status=web_data.Robot_2[1],Robot_2_Node=web_data.Robot_2[2])

	@app.route("/command", methods=['POST'])
	def command():
		web_data.text = request.form['text']
		web_data.update_rn()
		#processed_text = text
		print web_data.text
		return render_template('index.html',call=web_data.get_c(),
	                                        Robot_1_Name=web_data.Robot_1[0],Robot_1_Status=web_data.Robot_1[1],Robot_1_Node=web_data.Robot_1[2],
	                                        Robot_2_Name=web_data.Robot_2[0],Robot_2_Status=web_data.Robot_2[1],Robot_2_Node=web_data.Robot_2[2])

	@app.route('/submit', methods=['POST'])
	def submit():
		web_data.update_counter()
	        print "counter = : ",web_data.get_c()
	        return render_template('index.html',call=web_data.get_c(),
	                                        Robot_1_Name=web_data.Robot_1[0],Robot_1_Status=web_data.Robot_1[1],Robot_1_Node=web_data.Robot_1[2],
	                                        Robot_2_Name=web_data.Robot_2[0],Robot_2_Status=web_data.Robot_2[1],Robot_2_Node=web_data.Robot_2[2])

	@app.route('/reset', methods=['POST'])
	def reset():
		web_data.reset_flag = 1
		web_data.set_counter()
	        print "counter = : ",web_data.get_c()
	        return render_template('index.html',call=web_data.get_c(),
	                                        Robot_1_Name=web_data.Robot_1[0],Robot_1_Status=web_data.Robot_1[1],Robot_1_Node=web_data.Robot_1[2],
	                                        Robot_2_Name=web_data.Robot_2[0],Robot_2_Status=web_data.Robot_2[1],Robot_2_Node=web_data.Robot_2[2])

	app.run(host='0.0.0.0',debug=False,port=5000)


if __name__ == '__main__':
	# run web app
	if WORKING_STATION:
		add_thread = threading.Thread(target = apprun)
  		add_thread.start()
    	w = WIFI_MASTER()
	#  w.talker()
