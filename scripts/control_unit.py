#!/usr/bin/env python3

import cv2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from dynamic_reconfigure.server import Server
from shifted_line_sim_pkg.cfg import ControlUnitConfig

# global variables
vel_msg = Twist()

previous_time = 0.0

################### callback ###################

def dynamic_reconfigure_callback(config, level):
    global RC
    RC = config
    return config

def yaw_rate_callback(angular_z):
    if RC.enable_drive:
        # engage the line following algorithm
        vel_msg.linear.x = RC.speed
        vel_msg.angular.z = angular_z.data
        cmd_vel_pub.publish(vel_msg)
    else:
        stop_vehicle()

def time_report_callback(report):
    global time_elapsed_secs
    time_elapsed_secs = report.data
    return

################### helper functions ###################

def stop_vehicle():
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    cmd_vel_pub.publish(vel_msg)
    return

################### main ###################

if __name__ == "__main__":
    rospy.init_node("control_unit", anonymous=True)

    rospy.Subscriber("/yaw_rate", Float32, yaw_rate_callback)
    rospy.Subscriber("/sdt_report/time_secs", Float32, time_report_callback)

    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    dynamic_reconfigure_server = Server(ControlUnitConfig, dynamic_reconfigure_callback)

    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      pass
