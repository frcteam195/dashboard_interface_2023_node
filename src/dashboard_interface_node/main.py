"""
Class definition of the dashboard interface node.
"""

from threading import Thread

import json
import socket
import rospy

from ck_ros_msgs_node.msg import Autonomous_Configuration, Autonomous_Selection, Health_Monitor_Control, Health_Monitor_Status
from frc_robot_utilities_py_node.frc_robot_utilities_py import *

UDP_IP = "0.0.0.0"
UDP_RECV_PORT = 5807
UDP_SEND_PORT = 5806
BUFFER_SIZE = 1024

class DashboardInterfaceNode:
    """
    The Dashboard Interface Node.
    """
    def __init__(self) -> None:
        register_for_robot_updates()

        self.autonomous_configuration_subscriber = BufferedROSMsgHandlerPy(Autonomous_Configuration)
        self.autonomous_configuration_subscriber.register_for_updates("AutonomousConfiguration")

        self.health_status_subscriber = BufferedROSMsgHandlerPy(Health_Monitor_Status)
        self.health_status_subscriber.register_for_updates("HealthMonitorStatus")

        self.acknowledge_publisher = rospy.Publisher(name="HealthMonitorControl", data_class=Health_Monitor_Control, queue_size=50, tcp_nodelay=True)
        self.autonomous_selection_publisher = rospy.Publisher(name="AutonomousSelection", data_class=Autonomous_Selection, queue_size=50, tcp_nodelay=True)

        self.clients = []

        rospy.loginfo(f"UDP target IP: {UDP_IP}")
        rospy.loginfo(f"UDP target port: {UDP_RECV_PORT}")

        self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.dashboard_socket.setblocking(0)
        self.dashboard_socket.bind((UDP_IP, UDP_RECV_PORT))

        loop_thread = Thread(target=self.loop)
        loop_thread.start()

        rospy.spin()

        loop_thread.join(5)

    def loop(self) -> None:
        """
        Periodic function for the dashboard interface node.
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            try:
                (buffer, (address, port)) = self.dashboard_socket.recvfrom(BUFFER_SIZE)
                message = json.loads(buffer)

                if (address, UDP_SEND_PORT) not in self.clients:
                    rospy.loginfo(f"New Client: {address}")
                    self.clients.append((address, UDP_SEND_PORT))

                if message["type"] == "data":
                    selection_message = Autonomous_Selection()
                    selection_message.autonomous = message["autonomous"]["autonomous"]
                    self.autonomous_selection_publisher.publish(selection_message)

                    if message["acknowledge"]:
                        acknowledge_message = Health_Monitor_Control()
                        acknowledge_message.faults = []
                        acknowledge_message.acknowledge = True
                        self.acknowledge_publisher.publish(acknowledge_message)

            except:
                pass

            self.send_dashboard_packet()

            rate.sleep()

    def send_dashboard_packet(self):
        """
        Sends a packet of data to the dashboard.
        """
        robot_status_data = ""
        if robot_status is not None:
            robot_status_data = robot_status.get_message()

        hmi_updates_data = ""
        if hmi_updates.get() is not None:
            hmi_updates_data = {
                "drivetrain_forward_back": hmi_updates.get().drivetrain_fwd_back,
                "drivetrain_left_right": hmi_updates.get().drivetrain_left_right,
                "drivetrain_swerve_direction": hmi_updates.get().drivetrain_swerve_direction,
                "drivetrain_heading": hmi_updates.get().drivetrain_heading,
                "drivetrain_brake": hmi_updates.get().drivetrain_brake,
                "drivetrain_orientation": hmi_updates.get().drivetrain_orientation
            }

        autonomous_configuration = None
        if self.autonomous_configuration_subscriber.get() is not None:
            autonomous_configuration_message : Autonomous_Configuration = self.autonomous_configuration_subscriber.get()
            autonomous_configuration = autonomous_configuration_message.autonomous_options

        faults = []
        if self.health_status_subscriber.get() is not None:
            health_status_message = self.health_status_subscriber.get()
            for fault in health_status_message.faults:
                faults.append({"code": fault.code, "priority": fault.priority})

        packet = {
            "robot_status": robot_status_data,
            "hmi_updates": hmi_updates_data,
            "autonomous_configuration": autonomous_configuration,
            "faults": faults
        }

        for client in self.clients:
            self.dashboard_socket.sendto(json.dumps(packet).encode("utf-8"), client)
