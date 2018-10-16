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
from patrol_sim.srv import register, start_patrol, reach_goal, report_end

# from pymavlink import *
from pymavlink import mavutil
from pymavlink.mavutil import location
from threading import Lock

# patrol_list = [location(-35.363262, 149.165237, 614, 353),
#                location(-35.362019, 149.164398, 614, 353),
#                location(-35.363758, 149.164734, 614, 353),
#                location(-35.363682, 149.166473, 614, 353),
#                location(-35.361889, 149.166107, 614, 353),
#                location(-35.362175, 149.164978, 614, 353),
#                location(-35.363430, 149.165070, 614, 353),
#                location(-35.363129, 149.165909, 614, 353),
#                location(-35.362576, 149.165878, 614, 353), ]

patrol_list_12 = [location(-35.363258, 149.165237, 100.000000, 1),
               location(-35.365711, 149.166794, 100.000000, 1),
               location(-35.366482, 149.161835, 100.000000, 1),
               location(-35.361553, 149.161194, 100.000000, 1),
               location(-35.360596, 149.160751, 100.000000, 1),
               location(-35.359474, 149.162369, 100.000000, 1),
               location(-35.358913, 149.165512, 100.000000, 1),
               location(-35.361786, 149.166046, 100.000000, 1),
               location(-35.362057, 149.164444, 100.000000, 1),
               location(-35.363857, 149.164764, 100.000000, 1),
               location(-35.363686, 149.166336, 100.000000, 1),
               location(-35.362701, 149.165787, 100.000000, 1),]

patrol_list = patrol_list_12
allowed_time = 60 * 60


class PatrolPoint:
    def __init__(self, number):
        self.number = number
        self.idleness = 0.0
        self.min_idleness = 9999999999999999999.0
        self.max_idleness = 0.0
        self.avg_idleness = 0.0
        self.stddev_idleness = 0.0
        self.number_visit = 0.0
        self.last_visit = 0.0
        self.total_idleness = 0.0
        self.total2_idleness = 0.0
        self.avg_number_visit = 0.0
        self.min_number_visit = 999999999.0
        self.max_number_visit = 0.0
        self.taken = False


class StateReport:
    def __init__(self):
        self.battery_used = 0
        self.distance = 0
        self.point_visited = 0


class GlobalStateReport():
    def __init__(self):
        self.avg_battery_used = 0.0
        self.min_battery_used = 99999999999.0
        self.max_battery_used = 0.0
        self.avg_distance = 0.0
        self.min_distance = 99999999999999.0
        self.max_distance = 0.0
        self.avg_point_visited = 0.0
        self.min_point_visited = 999999999999.0
        self.max_point_visited = 0.0
        self.avg_time = 0.0
        self.min_time = 999999999999.0
        self.max_time = 0.0
        self.accu = 0.0
        self.lost_count = 0
        self.type = 0


class Monitor():
    def __init__(self):
        self.lock = Lock()
        self.lock2 = Lock()
        if rospy.has_param("~nb_drones"):
            self.nb_drones = rospy.get_param("~nb_drones")
            print("Number of drone expected %d" % self.nb_drones)
        else:
            self.nb_drones = 0
            print("NEED TO SET NUMBER OF DRONES")
            exit(0)
        if rospy.has_param("~nb_copter"):
            self.nb_copter = rospy.get_param("~nb_copter")
            print("Number of copter expected %d" % self.nb_copter)
        else:
            self.nb_copter = 0
        if rospy.has_param("~nb_rover"):
            self.nb_rover = rospy.get_param("~nb_rover")
            print("Number of rover expected %d" % self.nb_rover)
        else:
            self.nb_rover = 0
        if rospy.has_param("/strategy"):
            self.strategy = rospy.get_param("/strategy")
        else:
            self.strategy = "RANDOM"

        self.ns = rospy.get_namespace()
        self.drone_registered = [False] * self.nb_drones
        self.sim_time = 0.0
        self.start_time = -1.0
        self.time_sub = rospy.Subscriber("/simclock", Clock, self.get_sim_time)

        self.monitor_register_srv = rospy.Service("register", register, self.register)

        self.monitorlaunch_srv = rospy.Service("patrol_all", start_patrol, self.start_patrol_all)
        self.monitorgoal_srv = rospy.Service("reach_goal", reach_goal, self.reach_goal)
        self.monitorend_srv = rospy.Service("report_end", report_end, self.report_end)
        self.monitor_point_list = []

        for i in range(len(patrol_list)):
            self.monitor_point_list.append(PatrolPoint(i))
        self.globalPatrolPoint = PatrolPoint(99)
        self.globalReport_rover = GlobalStateReport()
        self.globalReport_copter = GlobalStateReport()
        self.simTim_pub = rospy.Publisher(self.ns + "doomclock_monitor", Float64, queue_size=1)
        self.start_time_pub = rospy.Publisher("/start_time", Float64, queue_size=1)
        self.hasPrint = False

    def register(self, req):
        with self.lock:
            if not self.drone_registered[req.drone]:
                print("Register %d", req.drone)
                self.drone_registered[req.drone] = True
            return True

    def get_sim_time(self, time):
        if not rospy.is_shutdown():
            self.sim_time = time.clock.secs

    def start_patrol_all(self, req):
        print("STARTING ALL")
        self.start_time = self.sim_time
        print("Start time %f" % self.start_time)
        for drone in range(0, self.nb_drones):
            if self.nb_drones > 1:
                prefix = "/drone" + str(drone)
            else:
                prefix = "/drone" + str(drone)
            start = rospy.ServiceProxy(prefix + "/start_patrol", start_patrol)
            print("STARTING DRONE %d" % drone)
            print(start(True))
        for i in range(len(patrol_list)):
            self.monitor_point_list[i].last_visit = self.sim_time - self.start_time
        self.globalPatrolPoint.min_idleness = 9999999999999
        return True

    def reach_goal(self, reach):
        with self.lock:
            current_time = reach.time

            print("Point reached %d" % reach.goal)
            print("At Time %f" % current_time)

            idleness = current_time - self.monitor_point_list[reach.goal].last_visit
            idleness = max(idleness, 0)  # remove negative value
            self.monitor_point_list[reach.goal].last_visit = current_time
            print("Idleness %f" % idleness)
            self.monitor_point_list[reach.goal].idleness = idleness
            self.globalPatrolPoint.max_idleness = max(idleness, self.globalPatrolPoint.max_idleness)
            self.monitor_point_list[reach.goal].max_idleness = max(idleness,
                                                                   self.monitor_point_list[reach.goal].max_idleness)
            self.globalPatrolPoint.min_idleness = min(idleness, self.globalPatrolPoint.min_idleness)
            self.monitor_point_list[reach.goal].min_idleness = min(idleness,
                                                                   self.monitor_point_list[reach.goal].min_idleness)

            # global stat
            self.globalPatrolPoint.number_visit += 1.0
            self.globalPatrolPoint.total_idleness += idleness
            self.globalPatrolPoint.total2_idleness += idleness * idleness

            # point stat
            self.monitor_point_list[reach.goal].number_visit += 1.0
            self.monitor_point_list[reach.goal].total_idleness += idleness
            self.monitor_point_list[reach.goal].total2_idleness += idleness * idleness
            self.monitor_point_list[reach.goal].avg_idleness = self.monitor_point_list[reach.goal].total_idleness / \
                                                               self.monitor_point_list[reach.goal].number_visit
            self.monitor_point_list[reach.goal].stddev_idleness = 1.0 / self.monitor_point_list[
                reach.goal].number_visit * math.sqrt(
                self.monitor_point_list[reach.goal].number_visit * self.monitor_point_list[reach.goal].total2_idleness -
                self.monitor_point_list[reach.goal].total_idleness * self.monitor_point_list[reach.goal].total_idleness)

            self.monitor_point_list[reach.goal].taken = False

            most_idle = 0
            most_idle_num = -1
            for i in range(len(patrol_list)):
                temp_idle = current_time - self.monitor_point_list[i].last_visit
                if temp_idle > most_idle and not self.monitor_point_list[i].taken:
                    most_idle = temp_idle
                    most_idle_num = i
            if most_idle_num == -1:
                print("No point free, random")
                most_idle_num = random.randint(0, len(patrol_list) - 1)
            self.monitor_point_list[most_idle_num].taken = True
            print("Send to %d" % most_idle_num)
            return most_idle_num

    def report_end(self, report):
        with self.lock2:
            if not report.lost:
                if report.type == 0:
                    self.globalReport_copter.type = 0
                    self.globalReport_copter.accu += 1
                    self.globalReport_copter.max_battery_used = max(report.battery_used, self.globalReport_copter.max_battery_used)
                    self.globalReport_copter.min_battery_used = min(report.battery_used, self.globalReport_copter.min_battery_used)
                    self.globalReport_copter.avg_battery_used += report.battery_used

                    self.globalReport_copter.max_distance = max(report.distance, self.globalReport_copter.max_distance)
                    self.globalReport_copter.min_distance = min(report.distance, self.globalReport_copter.min_distance)
                    self.globalReport_copter.avg_distance += report.distance

                    self.globalReport_copter.max_point_visited = max(report.point_visited, self.globalReport_copter.max_point_visited)
                    self.globalReport_copter.min_point_visited = min(report.point_visited, self.globalReport_copter.min_point_visited)
                    self.globalReport_copter.avg_point_visited += report.point_visited

                    self.globalReport_copter.max_time = max(report.time, self.globalReport_copter.max_time)
                    self.globalReport_copter.min_time = min(report.time, self.globalReport_copter.min_time)
                    self.globalReport_copter.avg_time += report.time
                if report.type == 1:
                    self.globalReport_rover.type = 1
                    self.globalReport_rover.accu += 1
                    self.globalReport_rover.max_battery_used = max(report.battery_used, self.globalReport_rover.max_battery_used)
                    self.globalReport_rover.min_battery_used = min(report.battery_used, self.globalReport_rover.min_battery_used)
                    self.globalReport_rover.avg_battery_used += report.battery_used

                    self.globalReport_rover.max_distance = max(report.distance, self.globalReport_rover.max_distance)
                    self.globalReport_rover.min_distance = min(report.distance, self.globalReport_rover.min_distance)
                    self.globalReport_rover.avg_distance += report.distance

                    self.globalReport_rover.max_point_visited = max(report.point_visited, self.globalReport_rover.max_point_visited)
                    self.globalReport_rover.min_point_visited = min(report.point_visited, self.globalReport_rover.min_point_visited)
                    self.globalReport_rover.avg_point_visited += report.point_visited

                    self.globalReport_rover.max_time = max(report.time, self.globalReport_rover.max_time)
                    self.globalReport_rover.min_time = min(report.time, self.globalReport_rover.min_time)
                    self.globalReport_rover.avg_time += report.time
            else:
                if report.type == 0:
                    self.globalReport_copter.lost_count += 1
                if report.type == 1:
                    self.globalReport_rover.lost_count += 1
            print("Get report")
            print("Total report copter %d" % self.globalReport_copter.accu)
            print("Total report rover %d" % self.globalReport_rover.accu)
            return True

    def print_final(self):
        self.globalPatrolPoint.avg_idleness = self.globalPatrolPoint.total_idleness / \
                                              self.globalPatrolPoint.number_visit
        self.globalPatrolPoint.stddev_idleness = 1.0 / self.globalPatrolPoint.number_visit * math.sqrt(
            self.globalPatrolPoint.number_visit * self.globalPatrolPoint.total2_idleness -
            self.globalPatrolPoint.total_idleness * self.globalPatrolPoint.total_idleness)
        for i in range(len(patrol_list)):
            self.globalPatrolPoint.avg_number_visit += self.monitor_point_list[i].number_visit
            self.globalPatrolPoint.min_number_visit = min(self.monitor_point_list[i].number_visit,
                                                          self.globalPatrolPoint.min_number_visit)
            self.globalPatrolPoint.max_number_visit = max(self.monitor_point_list[i].number_visit,
                                                          self.globalPatrolPoint.max_number_visit)
        self.globalPatrolPoint.avg_number_visit = self.globalPatrolPoint.avg_number_visit / len(patrol_list)

        total_accu = self.globalReport_copter.accu + self.globalReport_rover.accu
        total_avg_battery_used = (self.globalReport_rover.avg_battery_used + self.globalReport_copter.avg_battery_used)/ total_accu
        total_avg_distance = (self.globalReport_rover.avg_distance + self.globalReport_copter.avg_distance) / total_accu
        total_avg_point_visited = (self.globalReport_rover.avg_point_visited + self.globalReport_copter.avg_point_visited) / total_accu
        total_avg_time = (self.globalReport_rover.avg_time + self.globalReport_copter.avg_time) / total_accu

        if self.globalReport_rover.accu != 0:
            self.globalReport_rover.avg_battery_used = self.globalReport_rover.avg_battery_used / self.globalReport_rover.accu
            self.globalReport_rover.avg_distance = self.globalReport_rover.avg_distance / self.globalReport_rover.accu
            self.globalReport_rover.avg_point_visited = self.globalReport_rover.avg_point_visited / self.globalReport_rover.accu
            self.globalReport_rover.avg_time = self.globalReport_rover.avg_time / self.globalReport_rover.accu

        if self.globalReport_copter.accu != 0:
            self.globalReport_copter.avg_battery_used = self.globalReport_copter.avg_battery_used / self.globalReport_copter.accu
            self.globalReport_copter.avg_distance = self.globalReport_copter.avg_distance / self.globalReport_copter.accu
            self.globalReport_copter.avg_point_visited = self.globalReport_copter.avg_point_visited / self.globalReport_copter.accu
            self.globalReport_copter.avg_time = self.globalReport_copter.avg_time / self.globalReport_copter.accu

        print("globalReport Rover")
        print(self.globalReport_rover.__dict__)
        print("globalReport Copter")
        print(self.globalReport_copter.__dict__)
        print("globalPatrolPoint")
        print(self.globalPatrolPoint.__dict__)
        self.worst_avg_idleness = 0
        for i in range(len(patrol_list)):
            self.worst_avg_idleness = max(self.monitor_point_list[i].avg_idleness, self.worst_avg_idleness)
        print("worst_avg_idleness")
        print(self.worst_avg_idleness)
        import time
        timestr = time.strftime("%Y%m%d-%H%M%S")
        with open("simu_" + str(self.nb_drones) + "_copters_" + str(self.nb_copter) + "_rover_" + str(self.nb_rover) + "_" + str(self.strategy) + "_" + timestr + ".txt", "w") as f:
            f.write("avg_idleness : %f\n" % self.globalPatrolPoint.avg_idleness)
            f.write("min_idleness : %f\n" % self.globalPatrolPoint.min_idleness)
            f.write("max_idleness : %f\n" % self.globalPatrolPoint.max_idleness)
            f.write("stddev_idleness : %f\n" % self.globalPatrolPoint.stddev_idleness)
            f.write("worst_avg_idleness : %f\n" % self.worst_avg_idleness)

            f.write("min_number_visit : %f\n" % self.globalPatrolPoint.min_number_visit)
            f.write("max_number_visit : %f\n" % self.globalPatrolPoint.max_number_visit)
            f.write("avg_number_visit : %f\n" % self.globalPatrolPoint.avg_number_visit)
            f.write("total_number_visit : %f\n" % self.globalPatrolPoint.number_visit)

            f.write("copter avg_battery_used : %f\n" % self.globalReport_copter.avg_battery_used)
            f.write("copter min_battery_used : %f\n" % self.globalReport_copter.min_battery_used)
            f.write("copter max_battery_used : %f\n" % self.globalReport_copter.max_battery_used)

            f.write("copter avg_distance : %f\n" % self.globalReport_copter.avg_distance)
            f.write("copter min_distance : %f\n" % self.globalReport_copter.min_distance)
            f.write("copter max_distance : %f\n" % self.globalReport_copter.max_distance)

            f.write("copter avg_time : %f\n" % self.globalReport_copter.avg_time)
            f.write("copter min_time : %f\n" % self.globalReport_copter.min_time)
            f.write("copter max_time : %f\n" % self.globalReport_copter.max_time)

            f.write("copter avg_point_visited : %f\n" % self.globalReport_copter.avg_point_visited)
            f.write("copter min_point_visited : %f\n" % self.globalReport_copter.min_point_visited)
            f.write("copter max_point_visited : %f\n" % self.globalReport_copter.max_point_visited)
            f.write("copter lost : %d\n" % self.globalReport_copter.lost_count)

            f.write("rover avg_battery_used : %f\n" % self.globalReport_rover.avg_battery_used)
            f.write("rover min_battery_used : %f\n" % self.globalReport_rover.min_battery_used)
            f.write("rover max_battery_used : %f\n" % self.globalReport_rover.max_battery_used)

            f.write("rover avg_distance : %f\n" % self.globalReport_rover.avg_distance)
            f.write("rover min_distance : %f\n" % self.globalReport_rover.min_distance)
            f.write("rover max_distance : %f\n" % self.globalReport_rover.max_distance)

            f.write("rover avg_time : %f\n" % self.globalReport_rover.avg_time)
            f.write("rover min_time : %f\n" % self.globalReport_rover.min_time)
            f.write("rover max_time : %f\n" % self.globalReport_rover.max_time)

            f.write("rover avg_point_visited : %f\n" % self.globalReport_rover.avg_point_visited)
            f.write("rover min_point_visited : %f\n" % self.globalReport_rover.min_point_visited)
            f.write("rover max_point_visited : %f\n" % self.globalReport_rover.max_point_visited)
            f.write("rover lost : %d\n" % self.globalReport_rover.lost_count)

            f.write("avg_battery_used : %f\n" % total_avg_battery_used)
            f.write("avg_distance : %f\n" % total_avg_distance)
            f.write("avg_point_visited : %f\n" % total_avg_point_visited)
            f.write("avg_time : %f\n" % total_avg_time)

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            while not all(self.drone_registered) and not rospy.is_shutdown() and self.start_time < 0:
                rate.sleep()
            rospy.loginfo_once("ALL READY")
            self.simTim_pub.publish(allowed_time - (self.sim_time - self.start_time))
            self.start_time_pub.publish(self.start_time)
            if self.sim_time > self.start_time + allowed_time:
                if (self.globalReport_rover.accu + self.globalReport_copter.accu) == self.nb_drones:
                    self.print_final()
                    self.hasPrint = True
                    rospy.signal_shutdown("MISSION END. THANKS!")
            rate.sleep()


        if self.hasPrint is False:
            self.print_final()


if __name__ == '__main__':
    rospy.init_node("Monitor", anonymous=True)
    monitor = Monitor()
    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass

# worst_avg_idleness
# 497.5
# globalReport
# {'max_battery_used': 12346.0, 'avg_point_visited': 19, 'min_battery_used': 12346.0, 'min_point_visited': 19, 'accu': 1, 'avg_battery_used': 0, 'max_distance': 1684.0, 'avg_distance': 1684.0, 'max_point_visited': 19, 'min_distance': 1684.0}
# globalReport
# {'avg_idleness': 261.1578947368421, 'total2_idlness': 1875188.0, 'total_idlness': 4962.0, 'last_visit': 0, 'number_visit': 19.0, 'idleness': 0, 'min_idleness': 0, 'number': 99, 'max_idleness': 565.0, 'stddev_idleness': 174.6157475137291}
# worst_avg_idleness
