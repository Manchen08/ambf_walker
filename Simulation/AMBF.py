import rospy
import numpy as np
import ambf_msgs.msg as ambf
from Model import Human
from ambf_client import Client
import time


class AMBF(object):

    def __init__(self, mass, height):
        """

        :param name_space: name space of the the body
        """

        super(AMBF, self).__init__(mass, height)
        _client = Client()
        _client.connect()
        time.sleep(2)
        handle = _client.get_obj_handle('Hip')

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        self._time = value

    @property
    def dt(self):
        return self._dt

    @dt.setter
    def dt(self, value):
        self._dt = value

    def send_command(self, tau):
        cmd = ambf.ObjectCmd()
        cmd.publish_joint_names = True
        cmd.publish_joint_positions = True
        cmd.enable_position_controller = False
        cmd.joint_cmds = tau
        self.tau_pub.publish(cmd)

    def send_msg(self, msg):

        self.tau_pub.publish(msg)

    def joint_callback(self, msg):
        """
        callback fnc for the joint listener
        :type msg: ambf.ObjectState
        :param msg:
        :return:
        """

        # this checks if we have the joint states enable and if they are
        # not then they are are enables

        if not msg.joint_positions:
            msg = ambf.ObjectCmd()
            msg.publish_joint_names = True
            msg.publish_joint_positions = True
            self.tau_pub.publish(msg)
            return None

        q = np.array([0.0] * 7)
        qd = np.array([0.0] * 7)
        q[1:] = np.asarray(msg.joint_positions)
        q = np.round(q, 3)

        if self.time == 0:
            self.time = msg.sim_time

        qd = (q - self.q) / (msg.sim_time - self.time + 0.0000001)
        qd = np.clip(qd, -1.0, 1.0)
        self.dt = msg.wall_time - self.time
        self.time = msg.wall_time
        self.update_joints(q, qd)

