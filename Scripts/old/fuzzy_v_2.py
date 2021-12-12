#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import *
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from wpg.msg import fuzzy as fuzzy_msg, defuzzyfied as defuzzy_msg

from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler

import PySimpleGUI27 as sg
import thread
import time

gui = False
velocity_control_activate = False

#PID constantes
KPx = KPy = KPz = 2.95
KIx = KIy = KIz = 2.625
KDx = KDy = KDz = 0.7075
# Eu queria que esses valores definidos na interface ficassem salvas no fuzzy pra somar o ganho, mas que não se alterasse de forma recursiva

# non-brutness of fuzzy
SOFTNESS = 1  # SOFTEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEESSSS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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

            # rospy.loginfo("there is life on navigate")

            self.pub.publish(msg)
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
        def is_near(msg, _axi_1, _axi_2):
            # rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d", msg, _axi_1, _axi_2, abs(_axi_1 - _axi_2))
            return abs(_axi_1 - _axi_2) < 0.2

        if is_near('X', topic.pose.position.x, self.x) and \
            is_near('Y', topic.pose.position.y, self.y) and \
            is_near('Z', topic.pose.position.z, self.z):
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

        self.yaw_degrees = 0

        self.done_evt = threading.Event()

        # publisher for fuzzy (teste)
        self.pub_fuz = rospy.Publisher('fuzzy_values', fuzzy_msg, queue_size=10)
        # subscriber for fuzzy (teste)
        self.sub_fuz = rospy.Subscriber('defuzzy_values', defuzzy_msg, self.calculate_fuzzy)
        # publisher for reference position (ploting reasons)
        self.pub_refpos = rospy.Publisher('reference_pos', SP.PoseStamped, queue_size=10)
        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_velocity_cmd_vel(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'velocity_local'), SP.TwistStamped, self.velocity_meter)

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
                self.pose = 0
                self.oldpose = 0

            def calc(self, _setpos=0, _pos=0):
                self.old = self.__act
                self.__act = _setpos - _pos
                self.oldpose = self.pose
                self.pose = _pos

            @property
            def act(self):
                return self.__act

            # @act.setter
            # def act(self, value):
            #     self.old = self.__act
            #     self.__act = value

            @property
            def Intg(self):
                return self.__Intg

            @Intg.setter
            def Intg(self, value):
                _sum = self.__Intg + value
                self.__Intg = _sum if abs(_sum) < self.WindUp else self.__Intg

            @property
            def Deri(self):
                # return self.__act - self.old  # True Error derivative implementation
                return self.oldpose - self.pose # Derivative on Measurement implementation

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
        #todo implement Y and Z axis too
        self.defuzzed.P_X = fuz_topic.Delta_P_val
        self.defuzzed.I_X = fuz_topic.Delta_I_val
        self.defuzzed.D_X = fuz_topic.Delta_D_val

    def finish(self):
        self.activated = False
        # thread.exit_thread()

    def control_pid(self):
        msg = SP.TwistStamped(
            header=SP.Header(
                frame_id="Drone_Vel_setpoint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
        ref_pose_msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="setpoint_position",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )
        fuz_msg = fuzzy_msg(
            header=SP.Header(
                frame_id="fuzzy_values",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )

        while not rospy.is_shutdown():
            if not self.activated:
                break

            if self.fuzzyfy:
                #todo fuzzy com y e z
                rospy.loginfo_once("initalized fuzzyfication for the fist time")
                if not self.defuzzed.P_X == 0:
                    rospy.loginfo_once("primeiro valor de P_X modificado pelo sistema fuzzy")
                    rospy.loginfo_once("P_x = " + str(self.defuzzed.P_X))
                global KPx
                KPx = (2.95 + self.defuzzed.P_X)/SOFTNESS
                if KPx <=2.6: KPx=2.6
                if KPx >=3.3: KPx=3.3
                global KIx
                KIx = (2.625 + self.defuzzed.I_X)/SOFTNESS
                if KIx <= 0.25: KIx = 0.25
                if KIx >= 5.0: KIx = 5.0
                global KDx
                KDx = (0.7075 + self.defuzzed.D_X)/SOFTNESS
                if KDx <= 0.4: KDx = 0.4
                if KDx >= 0.85: KDx = 0.85

            self.error.x.calc(_setpos=self.x, _pos=DronePose.x)
            self.error.y.calc(_setpos=self.y, _pos=DronePose.y)
            self.error.z.calc(_setpos=self.z, _pos=DronePose.z)

            # rospy.loginfo(KPx)

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

            # rospy.loginfo(IY)

            self.x_vel = PX + DX + IX
            self.y_vel = PY + DY + IY
            self.z_vel = PZ + DZ + IZ

            fuz_msg.Error = self.error.x.act
            fuz_msg.D_Error = self.error.x.Deri
            fuz_msg.P_val = KPx
            fuz_msg.I_val = KIx
            fuz_msg.D_val = KDx

            ref_pose_msg.pose.position.x = setpoint_vel.x
            ref_pose_msg.pose.position.y = setpoint_vel.y
            ref_pose_msg.pose.position.z = setpoint_vel.z

            msg.twist.linear.x = self.x_vel
            msg.twist.linear.y = self.y_vel
            msg.twist.linear.z = self.z_vel

            fuz_msg.header.stamp = rospy.Time.now()
            self.pub_fuz.publish(fuz_msg)

            ref_pose_msg.header.stamp = rospy.Time.now()
            self.pub_refpos.publish(ref_pose_msg)

            msg.header.stamp = rospy.Time.now()
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

    def velocity_meter(self, topic):
        def is_near(msg, _axi_1, _axi_2):
            # rospy.loginfo("Velocity %s: local: %d, target: %d, abs diff: %d", msg, _axi_1, _axi_2, abs(_axi_1 - _axi_2))
            return abs(_axi_1 - _axi_2) < 0.2

        if is_near('X', DronePose.x, self.x) and \
            is_near('Y', DronePose.y, self.y) and \
            is_near('Z', DronePose.z, self.z):
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
        
        rospy.loginfo("## Iniciando modulo de controle de posição")
        setpoint_pos.init(0.0, 0.0, 0.0)
        setpoint_pos.start()

        rospy.loginfo("Set mode to offboard")
        modes.setMode("OFFBOARD")

        rospy.loginfo("Takeoff")
        setpoint_pos.set(0.0, 0.0, 2.0, wait=True)

        rospy.loginfo("## finalizando modulo de controle de posição")
        setpoint_pos.finish()

        rospy.loginfo("## Iniciando modulo de controle de velocidade")
        setpoint_vel.init(0.0, 0.0, 2.0)
        setpoint_vel.start()
        # velocity_control_activate = True

        _X_SIZE = 4
        _Y_SIZE = 4

        layout = [[sg.Graph(canvas_size=(400, 400), graph_bottom_left=(-200, -200), graph_top_right=(200, 200), background_color='red', key='graph', enable_events=True, drag_submits=True)],
                  [sg.Button("land", key="LAND"), sg.Button("go up", key="UP"), sg.Button("go down", key="DOWN")],
                  [sg.Button("ACTIVATE_FUZZY", key="ACTIVATE_FUZZY"), sg.Button("DEACTIVATE_FUZZY", key="DEACTIVATE_FUZZY")],
                  [sg.InputText("45", key="YAW"), sg.Button("SEND_YAW", key="SEND_YAW")],
                  [sg.InputText(KPx, size=(10,10), tooltip="K_P", key="K_P"),
                   sg.InputText(KIx, size=(10,10), tooltip="K_I", key="K_I"),
                   sg.InputText(KDx, size=(10,10), tooltip="K_D", key="K_D"), sg.Button("SEND_PID", key="SEND_PID")]]
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
                    #todo how the hell i send the yaw?
                    pass
                    # try:
                    #     setpoint_pos.yaw_degrees = int(values["YAW"])
                    # except ValueError:
                    #     print("That's not an int, stupid!")

                if event == 'SEND_PID':
                    KPx = KPy = KPz = float(values["K_P"])
                    KIx = KIy = KIz = float(values["K_I"])
                    KDx = KDy = KDz = float(values["K_D"])

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

        rospy.loginfo("Fly home")
        setpoint_vel.set(0.0, 0.0, 2.0, wait=True)

        rospy.loginfo("Landing")
        # Simulate a slow landing.
        setpoint_vel.set(0.0, 0.0, 1.0, wait=True)
        setpoint_vel.set(0.0, 0.0, 0.0, wait=True)
        setpoint_vel.set(0.0, 0.0, -0.1)
        modes.setMode("AUTO.LAND")

        rospy.loginfo("disarming")
        while state.armed:
            # print(extend_state.landed_state)
            if extend_state.landed_state == 1:
                modes.setArm(False)
            rate.sleep()

        setpoint_vel.finish()
        rospy.sleep(1)
        rospy.loginfo("Bye! XD")
    except rospy.ROSInterruptException:
        rospy.loginfo("Don't kill me this way please!! I may be a robot but have feelings );")
