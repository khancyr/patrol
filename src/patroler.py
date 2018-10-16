#!/usr/bin/env python

from __future__ import print_function

import rospy
from rosgraph_msgs.msg import Clock
import math
import mavros
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix, TimeReference
from std_msgs.msg import Header, Float64
from mavros_msgs.srv import *
from mavros.param import param_set, param_get_all, param_get

import random
from patrol_sim.srv import start_patrol, reach_goal, report_end, register

# from pymavlink import *
from pymavlink import mavutil
from pymavlink.mavutil import location
from monitor import patrol_list, StateReport, allowed_time
from threading import Lock


vehicle_type = ["Quadrotor", "Ground rover"]

HOME = mavutil.location(-35.363261, 149.165230, 584, 353)
MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE = ((1 << 0) | (1 << 1) | (1 << 2))
MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE = ((1 << 3) | (1 << 4) | (1 << 5))
MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE = ((1 << 6) | (1 << 7) | (1 << 8))
MAVLINK_SET_POS_TYPE_MASK_FORCE = (1 << 9)
MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE = (1 << 10)
MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE = (1 << 11)
MAV_FRAMES = {"MAV_FRAME_GLOBAL": mavutil.mavlink.MAV_FRAME_GLOBAL,
              "MAV_FRAME_GLOBAL_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
              "MAV_FRAME_GLOBAL_RELATIVE_ALT": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
              "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
              "MAV_FRAME_GLOBAL_TERRAIN_ALT": mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
              "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT": mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT}

battery_capacity = 12000


def longitude_scale(loc):
    scale = math.cos(math.radians(loc.lat * 1.0e-7))
    return min(1.0, max(0.01, scale))


def get_distance(loc1, loc2):
    """Get ground distance between two locations."""
    dlat = loc2.lat - loc1.lat
    try:
        dlong = (loc2.lng - loc1.lng)
    except AttributeError:
        dlong = (loc2.lon - loc1.lon)

    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_distance3(loc1, loc2):
    """Get ground distance between two locations."""
    dlat = loc2.lat - loc1.lat
    try:
        dlong = (loc2.lng - loc1.lng) * 1.113195e5
    except AttributeError:
        dlong = (loc2.lon - loc1.lon) * 1.113195e5
    dalt = max(0.001, abs(loc2.alt - loc1.alt) * 0.01)
    return math.sqrt((dlat * dlat) + (dlong * dlong) + (dalt * dalt))


strategy_list = ["RANDOM", "HIGHEST", "CYCLE"]


class PartrolSimCopter():
    def __init__(self, ):
        self.lock = Lock()
        self.model_name = "iris_demo"
        self.type = "Quadrotor"
        self.state = State()
        self.ns = rospy.get_namespace()
        mavros.set_namespace(mavros.DEFAULT_NAMESPACE)
        self.vehicle_type_sub = None
        if rospy.has_param("~instance"):
            self.instance = rospy.get_param("~instance")
        else:
            self.instance = 0
        self.targetLoc = None
        self.currentLoc = location(0.0, 0.0, 0.0, 0.0)
        self.currentRelAlt = 0.0
        self.reached = False
        self.reachedAlt = False
        self.start_patrol_srv = rospy.Service(self.ns + "start_patrol", start_patrol, self.start_patrol)
        self.patrol_list = patrol_list
        if rospy.has_param("~vehicle_name"):
            self.model_name = rospy.get_param("~vehicle_name")
        if rospy.has_param("/strategy"):
            temp = rospy.get_param("/strategy")
            if temp in strategy_list:
                self.strategy = temp
        else:
            self.strategy = "RANDOM"
        if self.strategy == "CYCLE" and self.instance > 0:
            random.shuffle(patrol_list)
            self.patrol_list = patrol_list
        self.timeout = allowed_time
        self.SM = None
        self.has_swap_batt = False
        self.tstart = None
        self.sim_time = 0
        self.takeoffAlt = 15.0 + (2.0 * self.instance)
        self.targetAlt = self.takeoffAlt
        self.hasInit = False
        self.pos_pub = rospy.Publisher(self.ns + "mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=2)
        self.pos_sub = rospy.Subscriber(self.ns + "mavros/global_position/global", NavSatFix, self.pos_reached)
        self.alt_sub = rospy.Subscriber(self.ns + "mavros/global_position/rel_alt", Float64, self.alt_reached)
        self.sub_state = rospy.Subscriber(self.ns + "mavros/state", State, self.state_callback)
        self.simTim_pub = rospy.Publisher(self.ns + "doomclock", Float64, queue_size=1)
        if self.instance == 0:
            self.time_sub = rospy.Subscriber(self.ns + "mavros/time_reference", TimeReference, self.get_time)
            self.clock_pub = rospy.Publisher("/simclock", Clock, queue_size=2)
        if self.instance > 0:
            self.time_sub = rospy.Subscriber("/simclock", Clock, self.get_sim_time)
        self.start_time_sub = rospy.Subscriber("/start_time", Float64, self.get_start_time)

        self.numberOfVisite = 0
        self.report = StateReport()
        self.report_end = False
        self.travelDistance = 0.0
        self.battery_consumed = 0.0
        self.targetNext = 0
        self.targetNum = 0
        self.wait_at_target_start = 0
        self.battery_capacity = battery_capacity
        self.is_lost = False
        self.lost_time = False

    def progress(self, text):
        """Display autotest progress text."""
        print("Patroler " + str(self.instance) + " : " + text)

    def get_start_time(self, stime):
        if not rospy.is_shutdown():
            if self.tstart is None and stime.data > 0:
                self.tstart = stime.data
                self.progress("time start : %f" % self.tstart)

    def get_sim_time(self, time_r):
        if not rospy.is_shutdown():
            old_time = self.sim_time
            self.sim_time = time_r.clock.secs
            if self.sim_time == old_time:
                if not self.lost_time:
                    self.lost_time = True
                    self.end_timer = rospy.get_time()
                if self.lost_time and (rospy.get_time() - self.end_timer) > 60:
                    self.SM = "END_MISSION"
            elif self.lost_time:
                self.lost_time = False

    def get_time(self, time_r):
        if not rospy.is_shutdown():
            self.sim_time = time_r.time_ref.secs
            if self.instance == 0:
                msg = Clock()
                msg.clock = rospy.Time(time_r.time_ref.secs, time_r.time_ref.nsecs)
                self.clock_pub.publish(msg)

    def get_battery_consumed(self, event):
        self.drain_mav()
        self.mav.mav.request_data_stream_send(self.instance, 0,
                                              mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                              2, 1)
        m = self.mav.recv_match(type="BATTERY_STATUS", blocking=True, timeout=5)
        if m is not None:
            self.battery_consumed = m.current_consumed
            #rospy.loginfo_throttle(30, "battery_consumed %f" % self.battery_consumed)
            with self.lock:
                if self.battery_consumed > self.battery_capacity - 3000 and self.SM != "CHANGE_BATTERY":
                    self.SM = "CHANGE_BATTERY"
                    self.progress("RETURN HOME TO CHANGE BATTERY")
        return True

    def start_patrol(self, req):
        self.progress("STARTING PATROL")
        self.mav.mav.request_data_stream_send(self.instance+1, 0,
                                              mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                                              2, 1)
        self.setRateAll()
        rospy.sleep(10.0)
        if self.SM != "CHANGE_BATTERY":
            self.SM = "TAKEOFF"
        return True

    def drain_mav(self):
        count = 0
        while self.mav.recv_match(type='SYSTEM_TIME', blocking=False) is not None:
            count += 1
        # self.progress("Drained %u messages from mav" % count)

    def get_mavlink_connection_going(self):
        # get a mavlink connection going
        connection_string = "tcp:" + "127.0.0.1:" + str(5763 + self.instance * 10)
        try:
            self.mav = mavutil.mavlink_connection(connection_string,
                                                  robust_parsing=True,
                                                  source_component=250)
        except Exception as msg:
            self.progress("Failed to start mavlink connection on %s: %s" %
                          (connection_string, msg,))
            raise

    #     self.mav.message_hooks.append(self.message_hook)
    #     self.mav.idle_hooks.append(self.idle_hook)

    def run(self):
        rospy.wait_for_service(self.ns + "mavros/set_stream_rate")
        rospy.wait_for_service("/register")
        # rospy.wait_for_message("/diagnostics", DiagnosticArray)
        self.get_mavlink_connection_going()
        self.mav.mav.request_data_stream_send(self.instance+1, 0,
                                              mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                                              2, 1)
        self.progress("Patroler init for %s" % self.ns)
        self.progress("Strategy : %s" % self.strategy)
        self.drain_mav()
        self.getAllParam()
        self.setRateAll()
        self.getAllParam()
        batt_capa = param_get("BATT_CAPACITY")
        if self.battery_capacity != batt_capa:
            param_set("BATT_CAPACITY", self.battery_capacity)
        rospy.Timer(rospy.Duration(10), self.get_battery_consumed)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            while self.tstart is None and not rospy.is_shutdown():
                self.register()
                rate.sleep()
            rate = rospy.Rate(20)
            if self.sim_time < self.tstart + self.timeout and self.SM != "END_MISSION":
                if self.SM == "TAKEOFF":
                    if self.state.mode != "GUIDED":
                        self.setGuidedMode()
                        rate.sleep()
                        continue
                    if not self.state.armed:
                        self.setArm()
                    else:
                        if self.type == "Quadrotor":
                            if self.state.system_status != 4:
                                self.setTakeoffMode()
                            if self.currentRelAlt >= 10.0:
                                self.progress("MISSION ALT REACHED, STARTING MISSION")
                                self.SM = "MISSION"
                                self.wait_at_target_start = self.sim_time
                        if self.type == "Ground rover":
                            self.progress("STARTING MISSION")
                            self.SM = "MISSION"
                            self.wait_at_target_start = self.sim_time
                if self.SM == "MISSION":
                    if self.targetLoc is None:
                        if self.sim_time > self.wait_at_target_start + 20:
                            if self.strategy == "RANDOM":
                                self.targetNum = random.randint(0, len(self.patrol_list) - 1)
                            if self.strategy == "HIGHEST":
                                self.targetNum = self.targetNext
                            if self.strategy == "CYCLE":
                                if self.targetNum < len(self.patrol_list) - 1:
                                    self.targetNum += 1
                                else:
                                    self.targetNum = 0
                            self.progress("Going to point : %d" % self.targetNum)
                            self.targetLoc = self.patrol_list[self.targetNum]
                            self.progress(str(self.targetLoc))
                        else:
                            rospy.loginfo_once("Patroler " + str(self.instance) + " : Waiting at target")

                    if not self.reached:
                        if self.targetLoc is not None:
                            self.set_position_global_int()
                        rate = rospy.Rate(5)
                    else:
                        if self.targetLoc is not None:
                            self.report.point_visited += 1
                        self.targetLoc = None
                        self.wait_at_target_start = self.sim_time

                if self.SM == "CHANGE_BATTERY":
                    if self.state.mode != "RTL":
                        self.setRTL()
                    rate = rospy.Rate(5)
                    if not self.state.armed:
                        if not self.has_swap_batt:
                            self.progress("CHANGING BATTERY ...")
                            batt_capa = param_get("BATT_CAPACITY")
                            self.battery_capacity = batt_capa + battery_capacity
                            param_set("BATT_CAPACITY", self.battery_capacity)
                            self.progress("New Capacity %f" % param_get("BATT_CAPACITY"))
                            self.has_swap_batt = True
                            self.wait_batt_timer = self.sim_time
                        else:
                            if self.sim_time > self.wait_batt_timer + 60:
                                self.SM = "TAKEOFF"
                                self.has_swap_batt = False
                                self.progress("BATTERY CHANGED !")
            else:
                rospy.loginfo_once("Patroler " + str(self.instance) + " : MISSION END")
                self.SM = "END_MISSION"

            if self.SM == "END_MISSION":
                if self.state.mode != "RTL":
                    self.setRTL()
                if not self.state.armed:
                    if not self.report_end:
                        self.forge_report()
                    else:
                        rospy.signal_shutdown("Patroler " + str(self.instance) + " : MISSION END. THANKS!")
            self.simTim_pub.publish(self.timeout - (self.sim_time - self.tstart))
            rate.sleep()

    def state_callback(self, state):
        backStateConnected = self.state.connected  # need to set the stream rate at start
        self.state = state
        self.state.connected = backStateConnected
        with self.lock:
            if state.mode == "RTL" and state.system_status == 5 and self.SM != "CHANGE_BATTERY":
                print(self.SM)
                self.SM = "END_MISSION"
                self.is_lost = True
                rospy.loginfo_throttle(10, "Patroler " + str(self.instance) + " : SOMETHING GET WRONG, ABORTING MISSION")
            if state.mode == "LAND" and state.system_status == 5 and self.SM != "CHANGE_BATTERY":
                print(self.SM)
                self.SM = "END_MISSION"
                self.is_lost = True
                rospy.loginfo_throttle(10, "Patroler " + str(self.instance) + " : SOMETHING GET WRONG, ABORTING MISSION")
        if self.state.connected is not state.connected or not self.hasInit:
            self.state.connected = state.connected
            # param_get_all(force_pull=True)
            #self.setRateAll()
            self.vehicle_type_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.type_callback)
            self.hasInit = True
        # setStream = param_get("SR0_EXTRA1") > 0 and param_get("SR1_EXTRA1") > 0
        # if not setStream:
        #     self.setRateAll()

    def setRateAll(self):
        self.progress("Setting Rate for %s" % self.ns)
        rospy.wait_for_service(self.ns + "mavros/set_stream_rate")

        set_rate = rospy.ServiceProxy(self.ns + "mavros/set_stream_rate", StreamRate)
        ret = False
        try:
            ret = set_rate(stream_id=StreamRateRequest.STREAM_ALL, message_rate=5, on_off=True)
        except rospy.ServiceException as ex:
            fault(ex)
        rate = 5
        if self.instance == 0:
            rate = 20
        try:
            ret = set_rate(stream_id=StreamRateRequest.STREAM_EXTRA1, message_rate=rate, on_off=True)
        except rospy.ServiceException as ex:
            fault(ex)
        self.getAllParam()
        return ret

    def getAllParam(self):
        rospy.wait_for_service(self.ns + "mavros/param/pull")
        try:
            pull = rospy.ServiceProxy(self.ns + "mavros/param/pull", ParamPull)
            ret = pull(force_pull=True)
        except rospy.ServiceException as ex:
            raise IOError(str(ex))
        if not ret.success:
            raise IOError("Request failed.")
        return ret

    def type_callback(self, diag):
        index1 = [diag.status.index(x) for x in diag.status if x.name == (self.ns[1:] + "mavros: Heartbeat")]
        if index1:
            index2 = [diag.status[index1[0]].values.index(x) for x in diag.status[index1[0]].values if
                      x.key == "Vehicle type"]
            if index2:
                self.type = diag.status[index1[0]].values[index2[0]].value
                if self.type in vehicle_type:
                    self.progress("Vehicle type : %s" % str(self.type))
                    self.vehicle_type_sub.unregister()

    def setRTL(self):
        self.progress("Setting RTL Mode")
        rospy.wait_for_service(self.ns + "mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy(self.ns + "mavros/set_mode", mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService(custom_mode="RTL")  # return true or false
        except rospy.ServiceException as e:
            self.progress("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

    def setGuidedMode(self):
        self.progress("Setting Guided Mode")
        rospy.wait_for_service(self.ns + "mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy(self.ns + "mavros/set_mode", mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService(custom_mode="GUIDED")  # return true or false
        except rospy.ServiceException as e:
            self.progress("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

    def setArm(self):
        self.progress("Arming")
        rospy.wait_for_service(self.ns + "mavros/cmd/arming")
        try:
            armService = rospy.ServiceProxy(self.ns + "mavros/cmd/arming", mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            self.progress("Service arm call failed: " + str(e))

    def setTakeoffMode(self):
        self.progress("Taking off")
        self.targetAlt = self.takeoffAlt
        rospy.wait_for_service(self.ns + "mavros/cmd/takeoff")
        try:
            takeoffService = rospy.ServiceProxy(self.ns + "mavros/cmd/takeoff", mavros_msgs.srv.CommandTOL)
            takeoffService(altitude=self.takeoffAlt, latitude=0, longitude=0, min_pitch=0, yaw=0)
        except rospy.ServiceException as e:
            self.progress("Service takeoff call failed: " + str(e))

    def set_position_global_int(self):
        """set position message in guided mode."""
        msg = GlobalPositionTarget(header=Header(frame_id="mavsetp", stamp=rospy.get_rostime()))
        msg.latitude = self.targetLoc.lat
        msg.longitude = self.targetLoc.lng
        msg.altitude = self.targetAlt
        msg.coordinate_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE | MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE | MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE | MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE
        self.pos_pub.publish(msg)

    def pos_reached(self, position):
        oldLoc = self.currentLoc
        self.currentLoc = location(position.latitude, position.longitude, position.altitude)
        if self.state.armed:
            self.travelDistance += get_distance3(oldLoc, self.currentLoc)
        # rospy.loginfo_throttle(10, "travelDistance %f" % self.travelDistance)
        if self.targetLoc is not None:
            delta = get_distance(self.currentLoc, self.targetLoc)
            # print(delta)
            if delta <= 3.0:
                if not self.reached:
                    self.report_reach_goal()
                self.reached = True

            else:
                self.reached = False
        else:
            self.reached = False

    def alt_reached(self, altitude):
        self.currentRelAlt = altitude.data
        if abs(self.currentRelAlt - self.targetAlt) <= 1.5:
            # print("TARGET ALT REACHED")
            self.reachedAlt = True

    def register(self):
        # rospy.wait_for_service("/register", register)
        try:
            registerService = rospy.ServiceProxy("/register", register)
            registerService(self.instance)
        except rospy.ServiceException as e:
            self.progress("Service register call failed: " + str(e))

    def report_reach_goal(self):
        try:
            reach_goalService = rospy.ServiceProxy("/reach_goal", reach_goal)
            response = reach_goalService(goal=self.targetNum, time=(self.sim_time - self.tstart))
            self.targetNext = response.newgoal
        except rospy.ServiceException as e:
            self.progress("Service reach_goal call failed: " + str(e))

    def forge_report(self):
        self.progress("MISSION SUCESS !!!!")
        self.report.battery_used = self.battery_consumed
        self.report.distance = self.travelDistance
        try:
            reportService = rospy.ServiceProxy("/report_end", report_end)
            self.report_end = reportService(point_visited=self.report.point_visited,
                                            battery_used=self.report.battery_used, distance=self.report.distance,
                                            time=(self.sim_time - self.tstart), lost=self.is_lost, type=vehicle_type.index(self.type))
            self.progress("Has report %s" % str(self.report_end.success))
        except rospy.ServiceException as e:
            self.progress("Service report_end call failed: " + str(e))


# Metric :
# Number of completed patrol cycle
# calcule avg_idleness[i], stddev_idleness[i], number_of_visits sur les points a visite
# 	fprintf(file,"\nNode idleness\n");
# 	fprintf(file,"   worst_avg_idleness (graph) = %.1f\n", worst_avg_idleness);
# 	fprintf(file,"   avg_idleness (graph) = %.1f\n", avg_graph_idl);
# 	fprintf(file,"   median_idleness (graph) = %.1f\n", median_graph_idl);
# 	fprintf(file,"   stddev_idleness (graph) = %.1f\n", stddev_graph_idl);
#	fprintf(file,"\nGlobal idleness\n");
# fprintf(file,"   min = %.1f\n", min_idleness);
# fprintf(file,"   avg = %.1f\n", gavg);
# fprintf(file,"   stddev = %.1f\n", gstddev);
# fprintf(file,"   max = %.1f\n", max_idleness);


if __name__ == '__main__':
    rospy.init_node("Patroler", anonymous=True)
    print("Welcome")
    patroler = PartrolSimCopter()
    try:
        patroler.run()
    except rospy.ROSInterruptException:
        pass
