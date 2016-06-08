#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

import rospy
import rospkg
import rosparam
import roslaunch

import time, threading

from robotnik_msgs.msg import State
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

DEFAULT_FREQ = 100.0
MAX_FREQ = 500.0


# Class Template of Robotnik component for Pyhton
class MapNavManager:

	def __init__(self, args):

		self.node_name_ = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq']
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name_,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ


		self.real_freq = 0.0

		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 / self.desired_freq
		# State msg to publish
		self.msg_state = State()
		# Timer to publish state
		self.publish_state_timer = 1
		
		
		# Flags para iniciar procesos
		self.run_slam_gmapping = False
		self.run_amcl = False
		self.run_move_base = False
		'''
		self.start_robot_ctrller = False
		self.start_teleop = False
		self.start_map_server = False
		'''
		# Flags para parar procesos
		self.stop_slam_gmapping = False
		self.stop_amcl = False
		self.stop_move_base = False
		
		#Status que se rellena con el process.is_alive
		self.is_slam_gmapping = False
		self.is_amcl = False
		self.is_map_server = False
		self.is_move_base = False
		self.is_robot_ctrllr = False
		self.is_teleop = False
		
		#Nombres de los procesos
		#self.slam_gmapping_process_name = ""
		self.slam_gmapping_process_name = "slam_gmapping.yaml"
		self.amcl_process_name = ""
		self.map_server_process_name = ""
		self.move_base_process_name = ""
		self.robot_ctrllr_process_name = ""
		self.teleop_process_name = ""
		
		
		#Objetos process
		self.gmapping_process = None
		self.amcl_process = None

		#Objetos nodo
		self.node_gmapping = None
		self.node_amcl = None
		self.launch = None

		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate) #Each second publish state
		
		#Objetos servicio
		self.run_mapping_srv = None #rospy.Service('~start_mapping_srv', Trigger, self.startMappingServiceCb)
		self.run_navigation_srv = None #rospy.Service('~start_navigation_srv', Trigger, self.startNavigationServiceCb)
		self.stop_mapping_srv = None
		self.stop_navigation_srv = None
	#===================================================================================================================================
	#===================================================================================================================================
	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		self.initialized = True

		#TODO self.launch = roslaunch.scriptapi.ROSLaunch()
		#TODO self.launch.start()
		self.launch = roslaunch.scriptapi.ROSLaunch()
		self.launch.start()


		return 0

	#===================================================================================================================================
	#===================================================================================================================================

	def loadYaml(self,filename,namespace):
		'''
			Load all the parameters on the parameter server
			@return: True if OK, False otherwise
		'''
		rospackage = rospkg.RosPack()
		file_path = rospackage.get_path('map_nav_manager') + "/config/" + filename
		parameters = rosparam.load_file(file_path)
		rosparam.upload_params(namespace,parameters[0][0][namespace])

	#===================================================================================================================================
	#===================================================================================================================================

	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0

		# Publishers
		self._state_publisher = rospy.Publisher('~state', State, queue_size=10)
		# Subscribers
		# topic_name, msg type, callback, queue_size
		# self.topic_sub = rospy.Subscriber('topic_name', Int32, self.topicCb, queue_size = 10)
		# Service Servers
		self.run_mapping_srv = rospy.Service('~run_mapping_srv', Trigger, self.startMappingServiceCb)
		self.run_navigation_srv = rospy.Service('~run_navigation_srv', Trigger, self.startNavigationServiceCb)

		# Service Clients
		# self.service_client = rospy.ServiceProxy('service_name', ServiceMsg)
		# ret = self.service_client.call(ServiceMsg)
		
		

		self.ros_initialized = True

		self.publishROSstate()

		return 0

	#===================================================================================================================================
	#===================================================================================================================================


	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name_)

		# TODO cerrar los procesos activos
		
		# Cancels current timers
		self.t_publish_state.cancel()

		self._state_publisher.unregister()

		self.initialized = False

		return 0

	#===================================================================================================================================
	#===================================================================================================================================


	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		
		#TODO cerrar los procesos activos
		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()

		self.ros_initialized = False

		return 0

	#===================================================================================================================================
	#===================================================================================================================================


	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False

		return 0

	#===================================================================================================================================
	#===================================================================================================================================


	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()

		if self.running:
			rospy.loginfo('%s::start: Already running' % self.node_name_)
			return 0

		self.running = True

		self.controlLoop()

		return 0

	#===================================================================================================================================
	#===================================================================================================================================


	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''

		while self.running and not rospy.is_shutdown():
			t1 = time.time()

			if self.state == State.INIT_STATE:
				self.initState()

			elif self.state == State.STANDBY_STATE:
				self.standbyState()

			elif self.state == State.READY_STATE:
				self.readyState()

			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()

			elif self.state == State.FAILURE_STATE:
				self.failureState()

			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()

			elif self.state == "MAPPING_STATE":
				self.mappingState()

			elif self.state == "NAVIGATION_STATE":
				self.navigationState()

			self.allState()

			t2 = time.time()
			tdiff = (t2 - t1)


			t_sleep = self.time_sleep - tdiff

			if t_sleep > 0.0:
				try:
					rospy.sleep(t_sleep)
				except rospy.exceptions.ROSInterruptException:
					rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name_)
					self.running = False

			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)

		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name_)

		return 0

	#===================================================================================================================================
	#===================================================================================================================================


	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''

		return 0

	#===================================================================================================================================
	#===================================================================================================================================


	def initState(self):
		'''
			Actions performed in init state
		'''

		if not self.initialized:
			self.setup()

		else:
			self.switchToState(State.STANDBY_STATE)


		return

	#===================================================================================================================================
	#===================================================================================================================================


	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		self.switchToState(State.READY_STATE)

		return

	#===================================================================================================================================
	#===================================================================================================================================

	def readyState(self):
		'''
			Actions performed in ready state
		'''
		#TODO

		if self.run_slam_gmapping:
			self.loadYaml(self.slam_gmapping_process_name,"slam_gmapping")
			self.startSlamGmappingNode()
			self.switchToState('MAPPING_STATE')


		elif self.is_amcl and self.is_move_base:
			self.switchToState('NAVIGATION_STATE')

		'''
		rosbridge_pkg = "rosbridge_server"
		rosbridge_exe = "rosbridge_websocket"
		rosbridge_process_name = "rosbridge_websocket"
		node_rosbridge = roslaunch.core.Node(rosbridge_pkg, rosbridge_exe, rosbridge_process_name)
		self.rosbridge_process = self.launch.launch(node_rosbridge)
		'''
		'''
		TODO robot controller
		robot_cntrllr_pkg = ""
		robot_cntrllr_exe = ""
		robot_cntrller_process_name = ""
		node_robot_cntrller = roslaunch.core.Node(robot_cntrllr_pkg)
		'''

		'''
		TODO teleop node
		robot_cntrllr_pkg = ""
		robot_cntrllr_exe = ""
		robot_cntrller_process_name = ""
		node_robot_cntrller = roslaunch.core.Node(robot_cntrllr_pkg)
		'''


		return

	#===================================================================================================================================
	#===================================================================================================================================

	def navigationState(self):

		return


	def mappingState(self):

		return


	def shutdownState(self):
		'''
			Actions performed in shutdown state
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)

		return

	#===================================================================================================================================
	#===================================================================================================================================

	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''

		return

	#===================================================================================================================================
	#===================================================================================================================================

	def failureState(self):
		'''
			Actions performed in failure state
		'''


		return

	#===================================================================================================================================
	#===================================================================================================================================

	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			rospy.loginfo('%s::switchToState: %s'%(self.node_name_, self.stateToString(self.state)))

		return

	#===================================================================================================================================
	#===================================================================================================================================

	def allState(self):
		'''
			Actions performed in all states
		'''
		self.rosPublish()

		return

	#===================================================================================================================================
	#===================================================================================================================================

	def stateToString(self, state):
		'''
			@param state: state to set
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'

		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'

		elif state == State.READY_STATE:
			return 'READY_STATE'

		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'

		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'

		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'

		elif state == "MAPPING_STATE":
			return 'MAPPING_STATE'

		elif state == "NAVIGATION_STATE":
			return 'NAVIGATION_STATE'

		else:
			return 'UNKNOWN_STATE'

		#===================================================================================================================================
		#===================================================================================================================================

	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		self.msg_state.state = self.state
		self.msg_state.state_description = self.stateToString(self.state)
		self.msg_state.desired_freq = self.desired_freq
		self.msg_state.real_freq = self.real_freq
		self._state_publisher.publish(self.msg_state)

		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()

	"""
	def topicCb(self, msg):
		'''
			Callback for inelfe_video_manager state
			@param msg: received message
			@type msg: std_msgs/Int32
		'''
		# DEMO
				rospy.loginfo('MapNavManager:topicCb')


	def serviceCb(self, req):
		'''
			ROS service server
			@param req: Required action
			@type req: std_srv/Empty
		'''
		# DEMO
				rospy.loginfo('MapNavManager:serviceCb')
	"""
	#===================================================================================================================================
	#===================================================================================================================================

	def startNavigationServiceCb(self, req):
		rospy.loginfo('called!')
		return TriggerResponse(True, "Successfull!")

	#===================================================================================================================================
	#===================================================================================================================================

	def startMappingServiceCb(self, req):
		'''
			ROS start mapping server
			@param req: Required action
			@type req: std_srv/Empty
		'''
		'''EXPLANATION: Here I should put a mecanism to start all the mapping stuff and changes some flags'''
		self.run_slam_gmapping = True
		rospy.loginfo('called!')
		return TriggerResponse(True, "Successfull!")
		'''
		if not self.is_slam_gmapping:
			gmapping_pkg = "gmapping"
			gmapping_exe = "slam_gmapping"
			gmapping_process_name = "slam_gmapping"
			node_gmapping = roslaunch.core.Node(gmapping_pkg, gmapping_exe, name=gmapping_process_name)
			self.gmapping_process = self.launch.launch(node_gmapping)
			self.is_slam_gmapping = True
			return TriggerResponse(True, "Successfull!")
		else:
			return TriggerResponse(False, "Mapping is already running")
	
		if not self.is_slam_gmapping:
			#Load slam_gmapping and params
			#self.is_slam_gmapping = True
			self.start_slam_gmapping_node()
		'''	
		

	# ===================================================================================================================================
	# ===================================================================================================================================

	def startSlamGmappingNode(self):
		if not self.is_slam_gmapping:
			gmapping_pkg = "gmapping"
			gmapping_exe = "slam_gmapping"
			gmapping_process_name = "slam_gmapping"
			self.node_gmapping = roslaunch.core.Node(gmapping_pkg, gmapping_exe, name=gmapping_process_name)
			self.gmapping_process = self.launch.launch(self.node_gmapping)
			self.is_slam_gmapping = True
		# print gmapping_process.is_alive()
		# gmapping_process.stop()

	# ===================================================================================================================================
	# ===================================================================================================================================

	def start_amcl_node(self):
		if not self.is_amcl:
			amcl_pkg = "amcl"
			amcl_exe = "amcl"
			amcl_process_name = "amcl"
			node_amcl = roslaunch.core.Node(amcl_pkg, amcl_exe, amcl_process_name)
			self.amcl_process = self.launch.launch(node_amcl)
			self.is_amcl = True

	#===================================================================================================================================
	#===================================================================================================================================

def main():

	rospy.init_node("map_nav_manager")

	_name = rospy.get_name().replace('/','')

	arg_defaults = {
		'topic_state': 'state',
		'desired_freq': DEFAULT_FREQ,
	}

	args = {}

	for name in arg_defaults:
		try:
			if rospy.search_param(name):
				args[name] = rospy.get_param('~%s'%(name)) # Adding ~ the name of the node, because the param has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))


	nav_map_manager_node = MapNavManager(args)

	rospy.loginfo('%s: starting'%(_name))

	nav_map_manager_node.start()


if __name__ == "__main__":
	main()
