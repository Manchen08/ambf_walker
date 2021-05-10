#!/usr/bin/env python
import smach
import rospy
# from ambf_walker.msg import DesiredJoints
import numpy as np
from os.path import dirname, join

from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner
#from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point


class TaskTrackingState(smach.State):
    """
    TaskTrackingState is the state for the AMBF walker to track an end-effector (toe) trajectory in the task space
        inherits from smach.State
    """

    def __init__(self, model_name, controller_name, outcomes=["task tracked"]):
        """
        Initializing the TaskTrack state
        :param model_name:      the AMBF model name for the subscriber and publisher
        :param controller_name: the AMBF controller name for publishing messages of type JointState

        """

        smach.State.__init__(self, outcomes=outcomes)
        self.runner = self._get_walker()                # TPGMMRunner for the toe trajectory
        self.model_name = model_name                    # input parameter

        # self.joint_state = JointState()               # jointState message, ! change to geometry.point ?

        # self.joint_cb = rospy.Subscriber(model_name)

        self.rate = rospy.Rate(50)                      # 50Hz rate
        self._controller_name = controller_name         # input parameter

        self.pub = rospy.Publisher(model_name + "_track_set_points", Point, queue_size=1)

        self.count = 0                                  # counter for stepping through the runner


    def execute(self, userdata):
        """
        Function to run upon entering this state
        :param userdata: data passed into the state
        :return: the outcome of the state when finished executing
        """

        runner_len = self.runner.get_length()

        while self.count < runner_len:

            # rospy.loginfo(self.model_name + " is at " + str(self.count))
            step_output = self.runner.step()            # step the runner
            rospy.loginfo(self.model_name + " output is " + str(step_output))

            pos = self.runner.x

            msg = Point()
            msg.x = pos[0]
            msg.y = pos[1]
            msg.z = 0

            self.pub.publish(msg)

            self.count += 1                             # up the count
            self.rate.sleep()                           # sleep to run the while loop at the specified rate

        # rospy.loginfo("Runner length is: " + str(runner_len))
        # rospy.loginfo(self.model_name + " EXECUTED " + self._controller_name)

        return "task tracked"



    def _get_walker(self):
        """
        Return the TPGMMRunner runner for the generalized toe trajectory from the training data
        """

        project_root = dirname(dirname(__file__))
        config_path = join(project_root, 'config/simpletest.pickle')
        # config_path = join(project_root, 'config/walk2.pickle')

        return TPGMMRunner.TPGMMRunner(config_path)