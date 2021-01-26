import abc
import numpy as np
import rbdl
import time
import rospy
from Filters import LowPass, MeanFilter
from threading import Thread
from GaitCore.Bio import Joint, Leg
from GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rbdl_server.srv import RBDLModel, RBDLModelAlignment
from rbdl_server.srv import RBDLInverseDynamics
from . import Model
class ModelServer(Model.Model):

    def __init__(self, client, model_name, joint_names, model_path):

        self._rbdl_model = None
        self._client = client
        self._model_name = model_name
        #"/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/ambf_models/lumped/lumped.yaml"
        self.make_dynamic_model(model_name, model_path )
        self._q = np.array([])
        self._qd = np.array([])
        self.tau = np.array([])
        self._joint_map = {}
        self._state = np.array([])
        self._handle = None
        self._joints_names = []
        self._selected_joint_names = joint_names
        self._updater = Thread(target=self.update)
        self._enable_control = False
        self.qd_filter = MeanFilter.MeanFilter(1)
        self.q_filter = MeanFilter.MeanFilter(1)
        self.sub_torque = rospy.Subscriber(self.model_name + "_joint_torque", JointState, self.torque_cb)
        self.q_pub = rospy.Publisher(self.model_name + "_q", Float32MultiArray, queue_size=1)
        self.dyn_srv = rospy.ServiceProxy('InverseDynamics', RBDLInverseDynamics)

    # @property
    # def rbdl_model(self):
    #     return self._rbdl_model

    # @rbdl_model.setter
    # def rbdl_model(self, value):
    #     self._rbdl_model = value

    @property
    def model_name(self):
        return self._model_name

    @model_name.setter
    def model_name(self, value):
        self._model_name = value

    def torque_cb(self, tau):
        self.update_torque(list(tau.effort))

    def update_torque(self, tau):
        """
        self.rbdl_model = self.dynamic_model()
        :type tau: List
        """
        self.tau = tau
        self._enable_control = True

    # def get_rbdl_model(self):
    #     return self._model

    @property
    def enable_control(self):
        return self._enable_control

    @enable_control.setter
    def enable_control(self, value):
        self._enable_control = value

    @property
    def handle(self):
        return self._handle

    @handle.setter
    def handle(self, value):
        self._handle = value

    @property
    def q(self):
        return self._q

    @q.setter
    def q(self, value):
        my_joints = []
        for joint in self._selected_joint_names:
            if joint in self._joints_names:
                my_joints.append(value[self._joints_names.index(joint)])
        self._q = self.q_filter.update(np.asarray(my_joints))

    @property
    def qd(self):
        return self._qd

    @qd.setter
    def qd(self, value):
        my_joints = []
        for joint in self._selected_joint_names:
            if joint in self._joints_names:
                my_joints.append(value[self._joints_names.index(joint)])
        self._qd = self.qd_filter.update(np.asarray(my_joints))

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    @abc.abstractmethod
    def make_dynamic_model(self, name, model_path): 
        """"
        use the RBDL server to create the model 
        """
        try:
            model_srv = rospy.ServiceProxy('CreateModel', RBDLModel)
            resp1 = model_srv(name, model_path)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def update(self):
        """

        :return:
        """
        rate = rospy.Rate(1000)  # 1000hz
        q_msg = Float32MultiArray()

        # get the joint map
        self._joints_names = self.handle.get_joint_names()
        try:
            model_srv = rospy.ServiceProxy('AMBF2RBDL', RBDLModelAlignment)
            resp1 = model_srv(self.model_name, [])           
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        self._joint_map = {resp1.names[i]: resp1.ids[i] for i in range(len(resp1.names))}
        joints_idx = []
            # print(self._joints_names)
        for joint in self._selected_joint_names:
            if joint in self._joints_names:
                joints_idx.append(self._joints_names.index(joint))
        # loop through and get the joint values
        # set the torques 
        while 1:
            self.q = self.handle.get_all_joint_pos()
            self.qd = self.handle.get_all_joint_vel()
            self.state = (self.q, self.qd)
            self._joint_num = self.q.size
            q_msg.data = self.q
            self.q_pub.publish(q_msg)
            if self._enable_control: 
               self.handle.set_multiple_joint_effort(self.tau, joints_idx)
                #set multiple joint pos
            rate.sleep()

    def ambf_to_rbdl(self, q):
        """
        make the order of the joints for the dynamics
        """

        names = self._joints_names
        joints_aligned = [0.0]*len(names)
        values = list(self._joint_map.values())
        q_new = [0.0]*len(names)

        for ii, name in enumerate(names):
            index = self._joint_map[name] - 1
            joints_aligned[index] = q[ii]

        return joints_aligned

    def rbdl_to_ambf(self, q):
        """
        reverse the order of the AMBF
        """
        
        names = self._joints_names
        joints_aligned = [0.0]*len(names)
        values = list(self._joint_map.values())
        q_new = [0.0]*len(names)

        for ii, name in enumerate(names):
            index = self._joint_map[name] - 1
            q_new[ii] = q[index]

        return q_new


    @abc.abstractmethod
    def fk(self):
        pass

    @abc.abstractmethod
    def update_state(self, q, qd):
        self.state = q + qd

    @abc.abstractmethod
    def calculate_dynamics(self, qdd):
        pass

    @abc.abstractmethod
    def calculate_torque(self):
        pass


    # def state(self, q, qd ):
    #     self.get_left_leg.hip.angle.z = q[0]
    #     self.get_left_leg.knee.angle.z = q[1]
    #     self.get_left_leg.ankle.angle.z = q[2]

    #     self.get_right_leg.hip.angle.z = q[3]
    #     self.get_right_leg.knee.angle.z = q[4]
    #     self.get_right_leg.ankle.angle.z = q[5]

    def get_right_leg(self):
        """
        :return:
        """
        return self._right_leg

    def get_left_leg(self):
        """
        :return:
        """
        return self._left_leg



def get_traj(q0, qf, v0, vf, tf, dt):

    b = np.array([q0, v0, qf, vf]).reshape((-1,1))
    A = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [1.0, tf, tf ** 2, tf ** 3],
                  [0.0, 1.0, 2 * tf, 3 * tf * 2]])

    x = np.linalg.solve(A, b)
    q = []
    qd = []
    qdd = []

    for t in np.linspace(0, tf, int(tf/dt)):
        q.append(x[0] + x[1] * t + x[2] * t * t + x[3] * t * t * t)
        qd.append(x[1] + 2*x[2] * t + 3*x[3] * t * t)
        qdd.append(2*x[2] + 6*x[3] * t)

    traj = {}
    traj["q"] = q
    traj["qd"] = qd
    traj["qdd"] = qdd
    return traj


def runge_integrator(model, y, h, tau):

    k1 = rhs(model, y, tau)
    k2 = rhs(model, y + 0.5 * h * k1,tau)
    k3 = rhs(model, y + 0.5 * h * k2,tau)
    k4 = rhs(model, y + h * k3,tau)

    return y + h / 6. * (k1 + 2. * k2 + 2. * k3 + k4)


def rhs(model, y, tau):

    dim = model.dof_count
    res = np.zeros(dim * 2)
    Q = np.zeros(model.q_size)
    QDot = np.zeros(model.qdot_size)
    QDDot = np.zeros(model.qdot_size)
    Tau = tau
    for i in range(0, dim):
        Q[i] = y[i]
        QDot[i] = y[i + dim]

    rbdl.ForwardDynamics(model, Q, QDot, Tau, QDDot)

    for i in range(0, dim):
        res[i] = QDot[i]
        res[i + dim] = QDDot[i]

    return res


def finite_differences(model, y, u, h=0.01):
    """ calculate gradient of plant dynamics using finite differences
    x np.array: the state of the system
    u np.array: the control signal
    """

    dof = u.shape[0]
    num_states = model.q_size*2

    A = np.zeros((num_states, num_states))
    B = np.zeros((num_states, dof))

    eps = 1e-4 # finite differences epsilon
    for ii in range(num_states):
        # calculate partial differential w.r.t. x
        inc_x = y.copy()
        inc_x[ii] += eps
        state_inc = runge_integrator(model, inc_x, h, u)
        dec_x = y.copy()
        dec_x[ii] -= eps
        state_dec = runge_integrator(model, dec_x, h, u)
        A[:, ii] = (state_inc - state_dec) / (2 * eps)

    for ii in range(dof):
        # calculate partial differential w.r.t. u
        inc_u = u.copy()
        inc_u[ii] += eps
        state_inc = runge_integrator(model, y, h, inc_u)
        dec_u = u.copy()
        dec_u[ii] -= eps
        state_dec = runge_integrator(model, y, h, dec_u)
        B[:, ii] = (state_inc - state_dec) / (2 * eps)

    return A, B