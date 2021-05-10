#!/usr/bin/env python3
import sys
# from os import sys, path
# sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import numpy as np

from Controller import ControllerNode, ExoControllerServer
from Model import ExoskeletonServer, Human
from StateMachines import StateMachine
import rospy
from ambf_client import Client
from Controller import DynController

if __name__ == '__main__':
    rospy.init_node("Base")
    _client = Client()
    _client.connect()
    rate = rospy.Rate(1000)

    robot_joints = ['ExoLeftHip', 'ExoLeftKnee', 'ExoLeftAnkle',
                    'ExoRightHip', 'ExoRightKnee', 'ExoRightAnkle', 'ExoHipCrutches']

    file_path = "/home/burningombre/catkin_ws/src/ambf_walker/ambf_models/lumped/lumped.yaml"

    LARRE = ExoskeletonServer.ExoskeletonServer(_client, "exo", robot_joints, file_path)
    controller_server = ExoControllerServer.ExoControllerServer(LARRE)

    # while True:
    #     fk = LARRE.fk()
    #     print(fk["right_hip"])
    # while True:
    #     LARRE.calculate_torque()
    LARRE.handle.set_rpy(0, 0, 0)
    LARRE.handle.set_pos(0.0, 0, 3.0)

    # machine = StateMachine.ExoStateMachine(LARRE)
    rospy.spin()