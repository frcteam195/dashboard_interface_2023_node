#!/usr/bin/env python3

from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from ck_ros_msgs_node.msg import AutonomousConfiguration
from threading import Thread
import tf2_ros
import rospy
import socket
import json

UDP_IP = "0.0.0.0"
UDP_PORT = 41234
BUFFER_SIZE = 1024
clients = []

autonomous_configuration_options = None

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.setblocking(0)
sock.bind((UDP_IP, UDP_PORT))

def receive_autonomous_configuration_options(new_autonomous_configuration_options):
    global autonomous_configuration_options
    autonomous_configuration_options = new_autonomous_configuration_options


def send(msg):
    for c in clients:
        sock.sendto(json.dumps(msg).encode("utf-8"), c)


def send_dashboard_packet():
    global autonomous_configuration_options
    global hmi_updates
    global robot_status

    robot_status_data = ""
    if robot_status is not None:
        robot_status_data = robot_status.get_message()

    hmi_updates_data = ""
    if hmi_updates.get() is not None:
        hmi_updates_data = {
            "drivetrain_forward_back": hmi_updates.get().drivetrain_fwd_back,
            "drivetrain_left_right": hmi_updates.get().drivetrain_left_right
        }

    autonomous_configuration = ""
    if autonomous_configuration_options is not None:
        
        autonomous_configuration = {
            "autonomous_options": autonomous_configuration_options.autonomous_options,
            "game_pieces": autonomous_configuration_options.game_pieces,
            "starting_positions": autonomous_configuration_options.starting_positions
        }

    send({
        "robot_status": robot_status_data,
        "hmi_updates": hmi_updates_data,
        "autonomous_configuration": autonomous_configuration,
        "drive_orientation": "robotOriented",
        "faults": ["Fire!", "Help!", ":'("]
        })


def loop():
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        try:
            message, address = sock.recvfrom(BUFFER_SIZE)

            if address not in clients:
                rospy.loginfo("New Client: " + str(address))
                clients.append(address)

            rospy.loginfo(message)

        except:
            pass

        send_dashboard_packet()

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    rospy.Subscriber("/AutonomousConfiguration", AutonomousConfiguration, receive_autonomous_configuration_options)

    t1 = Thread(target=loop)
    t1.start()

    rospy.spin()

    t1.join(5)
