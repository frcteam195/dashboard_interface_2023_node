#!/usr/bin/env python3
import rospy
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from ck_ros_msgs_node.msg import Autonomous_Configuration, Autonomous_Selection, Health_Monitor_Control, Health_Monitor_Status
from threading import Thread
import socket
import json

UDP_IP = "0.0.0.0"
UDP_PORT = 41234
BUFFER_SIZE = 1024
clients = []

autonomous_configuration_options = None
health_status_message = None

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.setblocking(0)
sock.bind((UDP_IP, UDP_PORT))


def receive_autonomous_configuration_options(new_autonomous_configuration_options):
    global autonomous_configuration_options
    autonomous_configuration_options = new_autonomous_configuration_options


def receive_health_monitor_status(status_message):
    global health_status_message
    health_status_message = status_message


def send(msg):
    for c in clients:
        sock.sendto(json.dumps(msg).encode("utf-8"), c)


def send_dashboard_packet():
    global autonomous_configuration_options
    global health_status_message
    global hmi_updates
    global robot_status

    robot_status_data = ""
    if robot_status is not None:
        robot_status_data = robot_status.get_message()

    hmi_updates_data = ""
    if hmi_updates.get() is not None:
        hmi_updates_data = {
            "drivetrain_forward_back": hmi_updates.get().drivetrain_fwd_back,
            "drivetrain_left_right": hmi_updates.get().drivetrain_left_right,
            "drivetrain_swerve_direction": hmi_updates.get().drivetrain_swerve_direction,
            "drivetrain_brake": hmi_updates.get().drivetrain_brake,
            "drivetrain_orientation": hmi_updates.get().drivetrain_orientation
        }

    autonomous_configuration = ""
    if autonomous_configuration_options is not None:
        autonomous_configuration = {
            "autonomous_options": autonomous_configuration_options.autonomous_options,
            "game_pieces": autonomous_configuration_options.game_pieces,
            "starting_positions": autonomous_configuration_options.starting_positions
        }

    faults = []
    if health_status_message is not None:
        for fault in health_status_message.faults:
            faults.append({"code": fault.code, "priority": fault.priority})

    packet = {
        "robot_status": robot_status_data,
        "hmi_updates": hmi_updates_data,
        "autonomous_configuration": autonomous_configuration,
        "faults": faults
    }

    send(packet)


def loop():
    rate = rospy.Rate(10)

    acknowledge_pub = rospy.Publisher(
        name="HealthMonitorControl", data_class=Health_Monitor_Control, queue_size=50, tcp_nodelay=True)

    autonomous_selection_publisher = rospy.Publisher(
        name="AutonomousSelection", data_class=Autonomous_Selection, queue_size=50, tcp_nodelay=True)

    while not rospy.is_shutdown():

        try:
            buffer, address = sock.recvfrom(BUFFER_SIZE)
            message = json.loads(buffer)

            if address not in clients:
                rospy.loginfo("New Client: " + str(address))
                clients.append(address)

            if message["type"] == "data":                
                selection_message = Autonomous_Selection()
                selection_message.starting_position = message["autonomous"]["position"]
                selection_message.game_piece = message["autonomous"]["game_piece"]
                selection_message.autonomous = message["autonomous"]["autonomous"]
                autonomous_selection_publisher.publish(selection_message)

                if message["acknowledge"]:
                    acknowledge_message = Health_Monitor_Control()
                    acknowledge_message.faults = []
                    acknowledge_message.acknowledge = True
                    acknowledge_pub.publish(acknowledge_message)

        except:
            pass

        send_dashboard_packet()

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    rospy.Subscriber("AutonomousConfiguration", Autonomous_Configuration, receive_autonomous_configuration_options)
    rospy.Subscriber("HealthMonitorStatus", Health_Monitor_Status, receive_health_monitor_status)

    t1 = Thread(target=loop)
    t1.start()

    rospy.spin()

    t1.join(5)
