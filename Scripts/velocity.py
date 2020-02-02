#!/usr/bin/env python

import thread
# import threading
import time
from math import *

import PySimpleGUI27 as sg
# import mavros
# import rospy
from mavros import setpoint as SP
from mavros.utils import *

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import quaternion_from_euler

gui = False
velocity_control_activate = False

class DronePosition:
	def __init__(self):
		self.x=0.0
		self.y=0.0
		self.z=0.0
DronePose = DronePosition()

class DroneVelocity:
	def __init__(self):
		self.x=0.0
		self.y=0.0
		self.z=0.0
DroneVel = DroneVelocity()

class fcuModes:
	def setTakeoff(self):
		rospy.wait_for_service('mavros/cmd/takeoff')
		try:
			takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = 3)
		except rospy.ServiceException, e:
			print "Service takeoff call failed: %s"%e

	def setArm(self, _to_arm):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(_to_arm)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def setMode(self, _mode):
		if _mode in ("STABILIZED",
					 "OFFBOARD",
					 "ALTCTL",
					 "POSCTL",
					 "AUTO.LAND"):
			rospy.wait_for_service('mavros/set_mode')
			try:
				flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
				while not state.mode == _mode:
					flightModeService(custom_mode = _mode)
					rate.sleep()
			except rospy.ServiceException, e:
				print "service set_mode call failed: %s. Offboard Mode could not be set." % e
modes = fcuModes()

class SetpointPosition:
	def init(self, _x, _y, _z):
		self.x = _x
		self.y = _y
		self.z = _z
		self.yaw_degrees = 0

		self.activated = True
		self.done_evt = threading.Event()

		# publisher for mavros/setpoint_position/local
		self.pub = SP.get_pub_position_local(queue_size=10)
		# subscriber for mavros/local_position/local
		self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.reached)

		try:
			thread.start_new_thread(self.navigate_position, ())
		except:
			fault("Error: Unable to start thread")

	def finish(self):
		self.activated = False

	def navigate_position(self):
		msg = SP.PoseStamped(
			header=SP.Header(
				frame_id="waypoint_to_go",  # no matter, plugin don't use TF
				stamp=rospy.Time.now()),    # stamp should update
		)

		while not rospy.is_shutdown():
			if not self.activated:
				break

			msg.pose.position.x = self.x
			msg.pose.position.y = self.y
			msg.pose.position.z = self.z

			yaw = radians(self.yaw_degrees)
			quaternion = quaternion_from_euler(0, 0, yaw)
			msg.pose.orientation = SP.Quaternion(*quaternion)

			self.pub.publish(msg)
			rate.sleep()

	def set(self, _x, _y, _z, delay=0, wait=False):
		self.done_evt.clear()
		self.x = _x
		self.y = _y
		self.z = _z

		if wait:
			while not self.done_evt.is_set() and not rospy.is_shutdown():
				rate.sleep()

		if delay > 0:
			time.sleep(delay)

	def reached(self, topic):
		def is_near(msg, _axi_1, _axi_2):
			rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d", msg, _axi_1, _axi_2, abs(_axi_1 - _axi_2))
			return abs(_axi_1 - _axi_2) < 0.3

		if is_near('X', topic.pose.position.x, self.x) and \
			is_near('Y', topic.pose.position.y, self.y) and \
			is_near('Z', topic.pose.position.z, self.z):
			self.done_evt.set()

		DronePose.x = topic.pose.position.x
		DronePose.y = topic.pose.position.y
		DronePose.z = topic.pose.position.z
setpoint_pos = SetpointPosition()

# class SetpointVelocity:
# 	"""
# 	This class sends position targets to FCU's position controller
# 	"""
#
# 	def __init__(self):
# 		self.x = 0.0
# 		self.y = 0.0
# 		self.z = 0.0
# 		self.yaw_degrees = 0
# 		self.done_evt = threading.Event()
#
# 		# publisher for mavros/setpoint_position/local
# 		self.pub = SP.get_pub_position_local(queue_size=10)
# 		# subscriber for mavros/local_position/local
# 		self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.reached)
#
# 		try:
# 			thread.start_new_thread(self.navigate_position, ())
# 		except:
# 			fault("Error: Unable to start thread")
#
# 	def navigate_position(self):
# 		msg = SP.PoseStamped(
# 			header=SP.Header(
# 				frame_id="waypoint_to_go",  # no matter, plugin don't use TF
# 				stamp=rospy.Time.now()),  # stamp should update
# 		)
#
# 		while not rospy.is_shutdown():
# 			if velocity_control_activate:
# 				break
#
# 			msg.pose.position.x = self.x
# 			msg.pose.position.y = self.y
# 			msg.pose.position.z = self.z
#
# 			yaw = radians(self.yaw_degrees)
# 			quaternion = quaternion_from_euler(0, 0, yaw)
# 			msg.pose.orientation = SP.Quaternion(*quaternion)
#
# 			self.pub.publish(msg)
# 			rate.sleep()
#
# 	def set(self, _x, _y, _z, delay=0, wait=False):
# 		self.done_evt.clear()
# 		self.x = _x
# 		self.y = _y
# 		self.z = _z
#
# 		if wait:
# 			while not self.done_evt.is_set() and not rospy.is_shutdown():
# 				rate.sleep()
#
# 		time.sleep(delay)
#
# 	def reached(self, topic):
# 		def is_near(msg, _axi_1, _axi_2):
# 			rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d", msg, _axi_1, _axi_2,
# 			               abs(_axi_1 - _axi_2))
# 			return abs(_axi_1 - _axi_2) < 0.3
#
# 		if is_near('X', topic.pose.position.x, self.x) and \
# 				is_near('Y', topic.pose.position.y, self.y) and \
# 				is_near('Z', topic.pose.position.z, self.z):
# 			self.done_evt.set()
#
# 		DronePose.x = topic.pose.position.x
# 		DronePose.y = topic.pose.position.y
# 		DronePose.z = topic.pose.position.z

state = State()
def stateCb1(msg):
	global state
	state= msg

extend_state = ExtendedState()
def stateCb2(msg):
	global extend_state
	extend_state = msg


if __name__ == '__main__':
	try:
		rospy.init_node('waypoint_controller')
		mavros.set_namespace()  # initialize mavros module with default namespace
		rate = rospy.Rate(20)

		rospy.Subscriber('mavros/state', State, stateCb1)
		rospy.Subscriber('mavros/extended_state', ExtendedState, stateCb2)

		rospy.loginfo("arming")
		while not state.armed:
			modes.setArm(True)
			rate.sleep()
		
		rospy.loginfo("initiate Pos publisher")
		# comecar a publicar mensagens de setpoint
		setpoint_pos.init(0.0, 0.0, 0.0)

		rospy.loginfo("Set mode to offboard")
		modes.setMode("OFFBOARD")
		
		rospy.loginfo("Takeoff")
		setpoint_pos.set(0.0, 0.0, 2.0, wait=True)

		# velocity_control_activate = True

		_X_SIZE = 4
		_Y_SIZE = 4

		layout = [[sg.Graph(canvas_size=(400, 400), graph_bottom_left=(-200, -200), graph_top_right=(200, 200), background_color='red', key='graph', enable_events=True, drag_submits=True)],
		          [sg.Button("land", key="LAND"), sg.Button("go up", key="UP"), sg.Button("go down", key="DOWN")],
		          [sg.InputText("45", key="YAW"), sg.Button("SEND_YAW", key="SEND_YAW")]]
		window = sg.Window('Drone control view', layout, finalize=True)
		graph = window['graph']

		oval1 = graph.draw_oval((-5, 0), (5, 50), fill_color='purple', line_color='purple')
		oval3 = graph.draw_oval((0, 5), (50, -5), fill_color='purple', line_color='purple')
		oval2 = graph.draw_oval((5, 0), (-5, -50), fill_color='blue', line_color='blue')
		oval4 = graph.draw_oval((0, -5), (-50, 5), fill_color='blue', line_color='blue')
		circleSize = 15
		circle = graph.draw_circle((0, 0), circleSize, fill_color='black', line_color='green')

		pointSize = 10
		point = graph.draw_point([0, 0], pointSize, color='green')

		gui = True

		while(True):
			event, values = window.Read(timeout=100)
			# print(event, values)

			if event in (None, 'LAND'):
				break

			if event == 'UP':
				setpoint_pos.z = setpoint_pos.z + 0.5

			if event == 'DOWN':
				setpoint_pos.z = setpoint_pos.z - 0.5

			if event == 'SEND_YAW':
				try:
					setpoint_pos.yaw_degrees = int(values["YAW"])
				except ValueError:
					print("That's not an int, stupid!")

			if not event == u'__TIMEOUT__':
				print(event, values)
				_x, _y = values["graph"]
				if not _x == None:
					graph.RelocateFigure(point, _x - pointSize, _y + pointSize)

					setpoint_pos.set(_X_SIZE * _x / 400.0, _Y_SIZE * _y / 400.0, setpoint_pos.z)

			graph.RelocateFigure(circle, DronePose.x * 25 * _X_SIZE - circleSize, DronePose.y * 25 * _Y_SIZE + circleSize)

			# print(DronePose.x, DronePose.y)

		rospy.loginfo("Fly home")
		setpoint_pos.set(0.0, 0.0, 2.0, wait=True)

		rospy.loginfo("Landing")
		# Simulate a slow landing.
		setpoint_pos.set(0.0, 0.0, 1.0, wait=True)
		setpoint_pos.set(0.0, 0.0, 0.0)
		setpoint_pos.set(0.0, 0.0, -0.2)
		modes.setMode("AUTO.LAND")

		rospy.loginfo("disarming")
		while state.armed:
			# print(extend_state.landed_state)
			if extend_state.landed_state == 1:
				modes.setArm(False)
			rate.sleep()

		rospy.loginfo("Bye!")
	except rospy.ROSInterruptException:
		pass
