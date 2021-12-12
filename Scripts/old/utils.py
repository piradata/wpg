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


class DronePosition:
	def __init__(self):
		self.x=0.0
		self.y=0.0
		self.z=0.0


class DroneVelocity:
	def __init__(self):
		self.x=0.0
		self.y=0.0
		self.z=0.0

class fcuModes:
	state = State()
	extend_state = ExtendedState()

	@classmethod
	def stateCb1(cls, msg):
		cls.state = msg

	@classmethod
	def stateCb2(cls, msg):
		cls.extend_state = msg

	def __init__(self):
		rospy.Subscriber('mavros/state', State, self.stateCb1)
		rospy.Subscriber('mavros/extended_state', ExtendedState, self.stateCb2)


	def setTakeoff(self):
		rospy.wait_for_service('mavros/cmd/takeoff')
		try:
			takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = 3)
		except rospy.ServiceException as e:
			print(f"Service takeoff call failed: {e}")

	def setArm(self, _to_arm):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(_to_arm)
		except rospy.ServiceException as e:
			print(f"Service arming call failed: {e}")

	def setMode(self, _mode):
		global rate
		if _mode in ("STABILIZED",
					 "OFFBOARD",
					 "ALTCTL",
					 "POSCTL",
					 "AUTO.LAND"):
			rospy.wait_for_service('mavros/set_mode')
			try:
				flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
				while not self.__class__.state.mode == _mode:
					flightModeService(custom_mode = _mode)
					rate.sleep()
			except rospy.ServiceException as e:
				print(f"service set_mode call failed: {e}. Offboard Mode could not be set.")

	def wait_for_land_and_disarm(self):
		global rate
		while self.__class__.state.armed:
			# print(extend_state.landed_state)
			if self.__class__.extend_state.landed_state == 1:
				self.setArm(False)
			rate.sleep()


class SetpointPosition:
	def init(self, _x, _y, _z):
		self.x = _x
		self.y = _y
		self.z = _z
		self.yaw_degrees = 0

		self.done_evt = threading.Event()

		# publisher for mavros/setpoint_position/local
		self.pub = SP.get_pub_position_local(queue_size=10)
		# subscriber for mavros/local_position/local
		self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.reached)

	def start(self):
		self.activated = True

		try:
			thread.start_new_thread(self.navigate_position, ())
		except:
			fault("Error: Unable to start thread")

	def finish(self):
		self.activated = False
		# thread.exit_thread()

	def navigate_position(self):
		global rate
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
		global rate
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
		global DronePose
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


class SetpointVelocity:
	def init(self, _x, _y, _z):
		self.x = _x
		self.y = _y
		self.z = _z
		self.x_vel = 0
		self.y_vel = 0
		self.z_vel = 0

		self.yaw_degrees = 0

		self.done_evt = threading.Event()

		# publisher for mavros/setpoint_position/local
		self.pub = SP.get_pub_velocity_cmd_vel(queue_size=10)
		# subscriber for mavros/local_position/local
		self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'velocity_local'), SP.TwistStamped, self.velocity_meter)

	def start(self):
		self.activated = True

		class ErrorObj(object):
			def __init__(self, InitValue=0, WindUp=5):
				self.__act = InitValue
				self.old = InitValue
				self.__Intg = 0
				self.WindUp = WindUp

			@property
			def act(self):
				return self.__act

			@act.setter
			def act(self, value):
				self.old = self.__act
				self.__act = value

			@property
			def Intg(self):
				return self.__Intg

			@Intg.setter
			def Intg(self, value):
				_sum = self.__Intg + value
				self.__Intg = _sum if abs(_sum) < self.WindUp else self.__Intg

			@property
			def Deri(self):
				return self.__act - self.old

		class ErrorVector():
			def __init__(self):
				self.x = ErrorObj(WindUp=4)
				self.y = ErrorObj(WindUp=4)
				self.z = ErrorObj(WindUp=2)

		self.error = ErrorVector()

		try:
			thread.start_new_thread(self.control_pid, ())
		except:
			fault("Error: Unable to start thread")

	def finish(self):
		self.activated = False
		# thread.exit_thread()

	def control_pid(self):
		global rate, DroneVel, DronePose
		msg = SP.TwistStamped(
			header=SP.Header(
				frame_id="Drone_Vel_setpoint",  # no matter, plugin don't use TF
				stamp=rospy.Time.now()),    # stamp should update
		)

		#PID constantes
		#2.1
		KPx = 2.15
		KPy = 2.15
		KPz = 2.15

		#0.4
		KDx = 0.38
		KDy = 0.38
		KDz = 0.38

		#0.25
		KIx = 0.27
		KIy = 0.27
		KIz = 0.27

		while not rospy.is_shutdown():
			if not self.activated:
				break

			#todo controle PID
			self.error.x.act = self.x - DronePose.x
			self.error.y.act = self.y - DronePose.y
			self.error.z.act = self.z - DronePose.z

			PX = self.error.x.act * KPx
			PY = self.error.y.act * KPy
			PZ = self.error.z.act * KPz

			_time_bet_run = (rate.sleep_dur.nsecs / 1000000000.0)

			DX = ((self.error.x.Deri) * float(KDx)) / _time_bet_run
			DY = ((self.error.y.Deri) * float(KDy)) / _time_bet_run
			DZ = ((self.error.z.Deri) * float(KDz)) / _time_bet_run

			self.error.x.Intg = self.error.x.act * KIx * _time_bet_run
			IX = self.error.x.Intg
			self.error.y.Intg = self.error.y.act * KIy * _time_bet_run
			IY = self.error.y.Intg
			self.error.z.Intg = self.error.z.act * KIz * _time_bet_run
			IZ = self.error.z.Intg

			#rospy.loginfo(PX, DX, IX, "\n", PY, DY, IY, "\n", PZ, DZ, IZ)
			#rospy.loginfo(PX, DX, IX)

			rospy.loginfo(IY)

			self.x_vel = PX + DX + IX
			self.y_vel = PY + DY + IY
			self.z_vel = PZ + DZ + IZ

			msg.twist.linear.x = self.x_vel
			msg.twist.linear.y = self.y_vel
			msg.twist.linear.z = self.z_vel

			# msg.twist.angular = SP.TwistStamped.twist.angular()

			self.pub.publish(msg)
			rate.sleep()

	def set(self, _x, _y, _z, delay=0, wait=False):
		global rate
		self.done_evt.clear()
		self.x = _x
		self.y = _y
		self.z = _z

		if wait:
			while not self.done_evt.is_set() and not rospy.is_shutdown():
				rate.sleep()

		if delay > 0:
			time.sleep(delay)

	def velocity_meter(self, topic):
		global DronePose, DroneVel
		def is_near(msg, _axi_1, _axi_2):
			rospy.logdebug("Velocity %s: local: %d, target: %d, abs diff: %d", msg, _axi_1, _axi_2, abs(_axi_1 - _axi_2))
			return abs(_axi_1 - _axi_2) < 0.3

		if is_near('X', DronePose.x, self.x) and \
			is_near('Y', DronePose.y, self.y) and \
			is_near('Z', DronePose.z, self.z):
			self.done_evt.set()

		DroneVel.x = topic.twist.linear.x
		DroneVel.y = topic.twist.linear.y
		DroneVel.z = topic.twist.linear.z









