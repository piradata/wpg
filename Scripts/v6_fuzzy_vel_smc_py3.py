#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from wpg.msg import fuzzy as fuzzy_msg
from wpg.msg import defuzzyfied as defuzzy_msg
from wpg.msg import vehicle_smc_gains as vehicle_smc_gains_msg

from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler

import math
import numbers
import PySimpleGUI27 as sg
import _thread as thread
import time

# PID constants
KPx = KPy = KPz = 2.6
KIx = KIy = KIz = 0.25
KDx = KDy = KDz = 0.4

# SMC constants (not in use yet)
K_roll, K_pitch, K_yaw = [0.23, 0.23, 0.10]
B_roll, B_pitch, B_yaw = [0.05, 0.05, 0.10]
L_roll, L_pitch, L_yaw = [20.0, 20.0, 10.0]

# non-brutness of fuzzy
SOFTNESS = 50

# set this variable to false to use the control panel
TEST_FLIGHT_MODE = True

# default reaching distance
DEFAULT_REACH_DIST = 0.1
DEFAULT_REACH_DIST_FOR_DEBUG = 0.05

# initial altitude
INITIAL_ALTITUDE = 0.5

# GAMBIT
FLY_INTERRUPTED = False

class DronePosition:
    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.z=0.0
    def xyz(self):
        return [self.x, self.x, self.x] 

class DroneVelocity:
    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.z=0.0

class fcuModes:
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
            except rospy.ServiceException as e:
                print(f"service set_mode call failed: {e}. Offboard Mode could not be set.")

class SetpointPosition:
    def init(self, _x, _y, _z):
        self.x = _x
        self.y = _y
        self.z = _z
        self.yaw_degrees = 0

        self.done_evt = threading.Event()

        # publisher for mavros/setpoint_position/local
        self.pub_setpoint = SP.get_pub_position_local(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub_pose = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.reached)

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
        msg_set_pos = SP.PoseStamped(
            header=SP.Header(
                frame_id="waypoint_to_go",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()
            ),    # stamp should update
        )
        while not rospy.is_shutdown():
            if not self.activated:
                break

            msg_set_pos.pose.position.x = self.x
            msg_set_pos.pose.position.y = self.y
            msg_set_pos.pose.position.z = self.z

            yaw = math.radians(self.yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg_set_pos.pose.orientation = SP.Quaternion(*quaternion)

            # rospy.loginfo("there is life on navigate")

            self.pub_setpoint.publish(msg_set_pos)
            rate.sleep()

    def set(self, _x, _y, _z, delay=0, wait=False):
        self.done_evt.clear()
        self.x = _x
        self.y = _y
        self.z = _z

        if wait:
            while not self.done_evt.is_set() and not rospy.is_shutdown():
                # rospy.loginfo("waiting")
                rate.sleep()

        if delay > 0:
            time.sleep(delay)

    def reached(self, topic):
        def is_near(_axi_1, _axi_2):
            # rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d", msg, _axi_1, _axi_2, abs(_axi_1 - _axi_2))
            return abs(_axi_1 - _axi_2) < 0.2

        if is_near(topic.pose.position.x, self.x) and \
            is_near(topic.pose.position.y, self.y) and \
            is_near(topic.pose.position.z, self.z):
            # rospy.loginfo("i have reached")
            # rospy.loginfo(self.done_evt)
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

        self.reaching_distance = DEFAULT_REACH_DIST

        self.yaw_degrees = 0

        self.done_evt = threading.Event()

        # publisher for fuzzy (test)
        self.pub_fuz = rospy.Publisher('fuzzy_values', fuzzy_msg, queue_size=10)
        # subscriber for fuzzy (test)
        self.sub_fuz = rospy.Subscriber('defuzzy_values', defuzzy_msg, self.calculate_fuzzy)

        # publisher for reference position (ploting reasons)
        self.pub_refpos = rospy.Publisher('reference_pos', SP.PoseStamped, queue_size=10)
        # publisher for mavros/setpoint_position/local
        self.pub_setpoint = SP.get_pub_velocity_cmd_vel(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub_vel = rospy.Subscriber(mavros.get_topic('local_position', 'velocity_local'), SP.TwistStamped, self.velocity_meter)

        # publisher for smc gains
        self.pub_smc = rospy.Publisher('smc_values', vehicle_smc_gains_msg, queue_size=10)

    def start(self):
        self.activated = True
        self.fuzzyfy = False

        class Fuzzy_diff:
            def __init__(self):
                self.P_X = 0
                self.I_X = 0
                self.D_X = 0
                self.P_Y = 0
                self.I_Y = 0
                self.D_Y = 0
                self.P_Z = 0
                self.I_Z = 0
                self.D_Z = 0

        class ErrorObj(object):
            def __init__(self, InitValue=0, WindUp=5.0):
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
                self.x = ErrorObj(WindUp=0.3)
                self.y = ErrorObj(WindUp=0.3)
                self.z = ErrorObj(WindUp=0.3)

        self.error = ErrorVector()

        self.defuzzed = Fuzzy_diff()

        try:
            thread.start_new_thread(self.control_pid, ())
        except:
            fault("Error: Unable to start thread")

    def activate_fuzzy(self):
        rospy.loginfo("initalized fuzzyfication")
        self.fuzzyfy = True

    def deactivate_fuzzy(self):
        rospy.loginfo("cancelled fuzzyfication")
        self.fuzzyfy = False

    def calculate_fuzzy(self, fuz_topic):
        # TODO: implement Y and Z axis too
        self.defuzzed.P_X = fuz_topic.Delta_P_val
        self.defuzzed.I_X = fuz_topic.Delta_I_val
        self.defuzzed.D_X = fuz_topic.Delta_D_val

    def finish(self):
        self.activated = False

    def control_pid(self):
        msg_set_vel = SP.TwistStamped(
            header=SP.Header(
                frame_id="Drone_Vel_setpoint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()
            ),    # stamp should update
        )
        ref_pose_msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="setpoint_position",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()
            ),  # stamp should update
        )
        fuz_msg = fuzzy_msg(
            header=SP.Header(
                frame_id="fuzzy_values",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()
            ),  # stamp should update
        )
        smc_msg = vehicle_smc_gains_msg(
            header=SP.Header(
                frame_id="smc_values",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()
            ),  # stamp should update
        )
        while not rospy.is_shutdown():
            if not self.activated:
                break

            if self.fuzzyfy:
                # TODO: fuzzy com y e z
                rospy.loginfo_once("initalized fuzzyfication for the fist time")
                if not self.defuzzed.P_X == 0:
                    rospy.loginfo_once("first value of P_X modified by fuzzy system")
                    rospy.loginfo_once(f"P_x = {str(self.defuzzed.P_X)}")
                global KPx
                KPx += self.defuzzed.P_X/SOFTNESS
                if KPx <= 0.6: KPx = 0.6
                if KPx >= 5.6: KPx = 5.6
                global KIx
                KIx += self.defuzzed.I_X/SOFTNESS
                if KIx <= 0.3: KIx = 0.3
                if KIx >= 4.5: KIx = 4.5
                global KDx
                KDx += self.defuzzed.D_X/SOFTNESS
                if KDx <= 0.1: KDx = 0.1
                if KDx >= 0.8: KDx = 0.8

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

            self.x_vel = PX + DX + IX
            self.y_vel = PY + DY + IY
            self.z_vel = PZ + DZ + IZ

            # topic for plotting desired position
            ref_pose_msg.pose.position.x = setpoint_vel.x
            ref_pose_msg.pose.position.y = setpoint_vel.y
            ref_pose_msg.pose.position.z = setpoint_vel.z

            msg_set_vel.twist.linear.x = self.x_vel
            msg_set_vel.twist.linear.y = self.y_vel
            msg_set_vel.twist.linear.z = self.z_vel

            fuz_msg.Error = self.error.x.act
            fuz_msg.D_Error = self.error.x.Deri
            fuz_msg.P_val = KPx
            fuz_msg.I_val = KIx
            fuz_msg.D_val = KDx

            smc_msg.k_gains = K_roll, K_pitch, K_yaw
            smc_msg.beta_gains = B_roll, B_pitch, B_yaw
            smc_msg.lambda_gains = L_roll, L_pitch, L_yaw

            # topic for plotting desired position
            ref_pose_msg.header.stamp = rospy.Time.now()
            self.pub_refpos.publish(ref_pose_msg)

            msg_set_vel.header.stamp = rospy.Time.now()
            self.pub_setpoint.publish(msg_set_vel)

            fuz_msg.header.stamp = rospy.Time.now()
            self.pub_fuz.publish(fuz_msg)

            smc_msg.header.stamp = rospy.Time.now()
            self.pub_smc.publish(smc_msg)

            rate.sleep()

    def set(self, _x, _y, _z, delay=0, wait=False, reaching_distance = None):
        if isinstance(reaching_distance, numbers.Number):
            self.reaching_distance = reaching_distance
        self.x = _x
        self.y = _y
        self.z = _z
        self.done_evt.clear()

        if wait:
            while not self.done_evt.is_set() and not rospy.is_shutdown():
                rate.sleep()

        # return reaching_distance to default value
        self.reaching_distance = DEFAULT_REACH_DIST

        if delay > 0:
            time.sleep(delay)

    def velocity_meter(self, topic):
        def is_near(_axi_1, _axi_2):
            return abs(_axi_1 - _axi_2) < self.reaching_distance

        # is_near considers a cuboid distance instead of a spherical one
        if is_near(DronePose.x, self.x) and \
            is_near(DronePose.y, self.y) and \
            is_near(DronePose.z, self.z):
            self.done_evt.set()

        DroneVel.x = topic.twist.linear.x
        DroneVel.y = topic.twist.linear.y
        DroneVel.z = topic.twist.linear.z

state = State()

def stateCb1(msg):
    global state
    state = msg

extend_state = ExtendedState()
def stateCb2(msg):
    global extend_state
    extend_state = msg

DronePose = DronePosition()
DroneVel = DroneVelocity()
modes = fcuModes()
setpoint_pos = SetpointPosition()
setpoint_vel = SetpointVelocity()


def land_n_disarm():
    global FLY_INTERRUPTED
    FLY_INTERRUPTED = True
    modes.setMode("AUTO.LAND")
    rospy.loginfo("disarming")
    while state.armed:
        # print(extend_state.landed_state)
        if extend_state.landed_state == 1:
            modes.setArm(False)
        rate.sleep()
    setpoint_vel.finish()
    rospy.sleep(1)

def test_run(in_X, in_Y, in_Z, dist):
    rospy.loginfo(f"++++++++ Begin test run")
    rospy.loginfo(f"++++++++ Origin: in_X -> {in_X}, in_Y -> {in_Y}, in_Z -> {in_Z}")

    _bgcs = 2
    rospy.loginfo(f"==== Move {_bgcs} meters")
    for position in range(int(_bgcs * 10000)):
        setpoint_vel.set(in_X + position/10000, in_Y, in_Z, wait=False, reaching_distance = dist * 2)
    setpoint_vel.set(in_X + _bgcs, in_Y, in_Z, wait=True, reaching_distance = dist)	
    rospy.sleep(2)
    rospy.loginfo(f"==== Make circle of radius {_bgcs}")
    for angle in range(360):
        setpoint_vel.set(in_X + _bgcs * math.cos(math.radians(angle)),
                         in_Y + _bgcs * math.sin(math.radians(angle)),
                         in_Z,
                         wait=True, reaching_distance = dist)
    rospy.sleep(2)
    rospy.loginfo("==== Return to origin")
    for position in range(int(_bgcs * 10000)):
        setpoint_vel.set(in_X + _bgcs - position/10000, in_Y, in_Z, wait=False, reaching_distance = dist * 2)
    setpoint_vel.set(in_X, in_Y, in_Z, wait=True, reaching_distance = dist)
    rospy.sleep(2)

    _smcs = 1
    rospy.loginfo(f"==== Make little circles of radius {_smcs}")
    for angle in range(360):
        setpoint_vel.set(in_X,
                         in_Y - _smcs + _smcs * math.cos(math.radians(angle)),
                         in_Z + _smcs * math.sin(math.radians(angle)),
                         wait=True, reaching_distance = dist)
    for angle in range(360):
        setpoint_vel.set(in_X - _smcs + _smcs * math.cos(math.radians(angle)),
                         in_Y,
                         in_Z + _smcs * math.sin(math.radians(angle)),
                         wait=True, reaching_distance = dist)
    for angle in range(360):
        setpoint_vel.set(in_X,
                         in_Y + _smcs - _smcs * math.cos(math.radians(angle)),
                         in_Z + _smcs * math.sin(math.radians(angle)),
                         wait=True, reaching_distance = dist)
    for angle in range(360):
        setpoint_vel.set(in_X + _smcs - _smcs * math.cos(math.radians(angle)),
                         in_Y,
                         in_Z + _smcs * math.sin(math.radians(angle)),
                         wait=True, reaching_distance = dist)

    _altd = 2
    _wttm = 20
    for direction in range(3):
        d_a = [0, 0, 0]
        d_a[direction] = 1 
        for _ in range(2):
            rospy.loginfo(f"==== Move {_altd} meters on dir {direction}")
            for position in range(int(_altd * 10000)):
                setpoint_vel.set(in_X + d_a[0] * position/10000,
                                 in_Y + d_a[1] * position/10000,
                                 in_Z + d_a[2] * position/10000,
                                 wait=False, reaching_distance = dist * 2)
            setpoint_vel.set(in_X + d_a[0] * _altd,
                             in_Y + d_a[1] * _altd,
                             in_Z + d_a[2] * _altd,
                             wait=True, reaching_distance = dist)
            rospy.loginfo(f"==== Wait for {_wttm} seconds")
            rospy.sleep(_wttm)
            rospy.loginfo("==== Return to origin")
            for position in range(int(_altd * 10000)):
                setpoint_vel.set(in_X + d_a[0] * _altd - d_a[0] * position/10000,
                                 in_Y + d_a[1] * _altd - d_a[1] * position/10000,
                                 in_Z + d_a[2] * _altd - d_a[2] * position/10000,
                                 wait=False, reaching_distance = dist * 2)
            setpoint_vel.set(in_X,
                             in_Y,
                             in_Z,
                             wait=True, reaching_distance = dist)
            rospy.loginfo(f"==== Wait for {_wttm} seconds")
            rospy.sleep(_wttm)

def test_run_goto(trgt_X, trgt_Y, trgt_Z, alt, dist):
    rospy.loginfo(f"++++++++ Begin test run")
    orig_X = DronePose.x
    orig_Y = DronePose.y
    orig_Z = DronePose.z
    rospy.loginfo(f"++++++++ Origin: orig_X -> {orig_X}, orig_Y -> {orig_Y}, orig_Z -> {orig_Z}")
    rospy.loginfo(f"++++++++ Target: trgt_X -> {trgt_X}, trgt_Y -> {trgt_Y}, trgt_Z -> {trgt_Z}")

    rospy.loginfo(f"==== Go up {alt} meters")
    setpoint_vel.set(orig_X, orig_Y, orig_Z + alt, wait=True, reaching_distance = dist)	
    rospy.loginfo("==== Move to target")
    setpoint_vel.set(trgt_X, trgt_Y, orig_Z + alt, wait=True, reaching_distance = dist)
    rospy.loginfo("==== Land")
    setpoint_vel.set(trgt_X, trgt_Y, trgt_Z, wait=True, reaching_distance = dist)
    rospy.loginfo(f"++++++++ End test run")

    land_n_disarm()


if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_controller')
        mavros.set_namespace()  # initialize mavros module with default namespace
        rate = rospy.Rate(20)

        state = State()
        rospy.Subscriber('mavros/state', State, stateCb1)
        extend_state = ExtendedState()
        rospy.Subscriber('mavros/extended_state', ExtendedState, stateCb2)

        rospy.loginfo("arming")
        while not state.armed:
            modes.setArm(True)
            rate.sleep()
        
        rospy.loginfo("## Initiating module of position control")
        setpoint_pos.init(0.0, 0.0, 0.0)
        setpoint_pos.start()

        rospy.loginfo("Set mode to offboard")
        modes.setMode("OFFBOARD")

        rospy.loginfo("Takeoff")
        setpoint_pos.set(0.0, 0.0, INITIAL_ALTITUDE, wait=True)

        rospy.loginfo("## finalizing module of position control")
        setpoint_pos.finish()

        rospy.loginfo("## Initiating module of velocity control")
        setpoint_vel.init(0.0, 0.0, INITIAL_ALTITUDE)
        setpoint_vel.start()
        setpoint_vel.set(0.0, 0.0, INITIAL_ALTITUDE, wait=True)

        if TEST_FLIGHT_MODE:
            rospy.loginfo("## Initiating test flight")
            # test_run(0.0, 0.0, 2.0, 0.05)
            # test_run_goto(-8.0, -318.0, -36.0, 20, 0.05)
            test_run_goto(80.0, 80.0, 1.0, 30, 0.1)
            rospy.loginfo("## Test flight finished!!!")
        else:
            rospy.loginfo("## Opening control interface")

            _X_SIZE = 4
            _Y_SIZE = 4

            layout = [
                    [sg.Graph(canvas_size=(400, 400),
                              graph_bottom_left=(-200, -200),
                              graph_top_right=(200, 200),
                              background_color='red',
                              key='graph',
                              enable_events=True,
                              drag_submits=True)],
                    [
                        sg.Button("land", key="LAND"),
                        sg.Button("go up", key="UP"),
                        sg.Button("go down", key="DOWN")
                    ],
                    [
                        sg.Button("ACTIVATE_FUZZY", key="ACTIVATE_FUZZY"),
                        sg.Button("DEACTIVATE_FUZZY", key="DEACTIVATE_FUZZY")
                    ],
                    [
                        sg.InputText("45", key="YAW"),
                        sg.Button("SEND_YAW", key="SEND_YAW")
                    ],
                    [
                        sg.InputText(KPx, size=(10,10), tooltip="K_P", key="K_P"),
                        sg.InputText(KIx, size=(10,10), tooltip="K_I", key="K_I"),
                        sg.InputText(KDx, size=(10,10), tooltip="K_D", key="K_D"),
                        sg.Button("SEND_PID", key="SEND_PID")
                    ]
                ]
            window = sg.Window('Drone control view', layout, finalize=True)
            graph = window['graph']

            oval1 = graph.draw_oval((-5, 0), (5,   50), fill_color='purple', line_color='purple')
            oval3 = graph.draw_oval((0,  5), (50,  -5), fill_color='purple', line_color='purple')
            oval2 = graph.draw_oval((5,  0), (-5, -50), fill_color='blue',   line_color='blue')
            oval4 = graph.draw_oval((0, -5), (-50,  5), fill_color='blue',   line_color='blue')
            circleSize = 15
            circle = graph.draw_circle((0, 0), circleSize, fill_color='black', line_color='green')

            pointSize = 10
            point = graph.draw_point([0, 0], pointSize, color='green')

            while(True):
                try:
                    event, values = window.Read(timeout=100)
                    # print(event, values)

                    if event in (None, 'LAND'):
                        break

                    if event == 'UP':
                        setpoint_vel.z = setpoint_vel.z + 1.0

                    if event == 'DOWN':
                        setpoint_vel.z = setpoint_vel.z - 1.0

                    if event == 'SEND_YAW':
                        # TODO: Implement
                        pass
                        # try:
                        # 	setpoint_pos.yaw_degrees = int(values["YAW"])
                        # except ValueError:
                        # 	print("That's not an int, stupid!")

                    if event == 'SEND_PID':
                        KPx = KPy = KPz = float(values["K_P"])
                        KIx = KIy = KIz = float(values["K_I"])
                        KDx = KDy = KDz = float(values["K_D"])

                    if event == 'SEND_SMC':
                        K_roll, K_pitch, K_yaw = list(map(float, values["SMC_K"].split(',')))
                        L_roll, L_pitch, L_yaw = list(map(float, values["SMC_L"].split(',')))
                        B_roll, B_pitch, B_yaw = list(map(float, values["SMC_B"].split(',')))

                    if event == 'ACTIVATE_FUZZY':
                        setpoint_vel.activate_fuzzy()

                    if event == 'DEACTIVATE_FUZZY':
                        setpoint_vel.deactivate_fuzzy()

                    if not event == u'__TIMEOUT__':
                        # print(event, values)
                        _x, _y = values["graph"]
                        if not _x == None:
                            # move setpoint on graph
                            graph.RelocateFigure(point, _x - pointSize, _y + pointSize)

                            setpoint_vel.set(_X_SIZE * _x / 400.0, _Y_SIZE * _y / 400.0, setpoint_vel.z)

                    # move drone on graph
                    graph.RelocateFigure(circle, DronePose.x * 25 * _X_SIZE - circleSize, DronePose.y * 25 * _Y_SIZE + circleSize)
                except KeyboardInterrupt:
                    pass

        if not FLY_INTERRUPTED:
            rospy.loginfo("Fly home")
            setpoint_vel.set(0.0, 0.0, INITIAL_ALTITUDE, wait=True)

            rospy.loginfo("Landing")
            # Simulate a slow landing.
            for altitude in range(int(INITIAL_ALTITUDE * 10)):
                setpoint_vel.set(0.0, 0.0, INITIAL_ALTITUDE - altitude/10, wait=True)
            setpoint_vel.set(0.0, 0.0, -0.1)
            land_n_disarm()
        rospy.loginfo("Bye! XD")
    except rospy.ROSInterruptException:
        rospy.loginfo("Don't kill me this way please!! I may be a robot but have feelings );")
