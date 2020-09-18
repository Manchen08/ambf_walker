

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import Model
from . import PDController
from sensor_msgs.msg import JointState
from . import ControllerBase
from GaitAnaylsisToolkit.LearningTools.Models import ModelBase
import rbdl
import copy
import time


class MPController(ControllerBase.BaseController):

    def __init__(self, model, runner):
        """

        :param model:
        :param kp:
        :param kd:
        """
        self._runner = runner
        self.expData = self._runner.get_expData()

        self.max_steps = 0
        self.step = 0
        super(MPController, self).__init__(model)
        self.rbdl_model = self._model._model
        self.initilzie()
        self.u = np.array([0, 0, 0, 0, 0, 0, 0])
        self.K2, self.tau, self.J_ = self.run_iLQR()
        self._x = self._runner.get_start()
        self._dx = np.zeros(len(self._x)).reshape((-1, 1))
        self._ddx = np.zeros(len(self._x)).reshape((-1, 1))

        self._kp = 50.0
        self._kc = 10.0
        Kp = np.zeros((7, 7))
        Kd = np.zeros((7, 7))

        Kp_hip = 100.0
        Kd_hip = 0.5

        Kp_knee = 125.0
        Kd_knee = 1.0

        Kp_ankle = 100.0
        Kd_ankle = 0.4

        Kp[0, 0] = Kp_hip
        Kd[0, 0] = Kd_hip
        Kp[1, 1] = Kp_knee
        Kd[1, 1] = Kd_knee
        Kp[2, 2] = Kp_ankle
        Kd[2, 2] = Kd_ankle

        Kp[3, 3] = Kp_hip
        Kd[3, 3] = Kd_hip
        Kp[4, 4] = Kp_knee
        Kd[4, 4] = Kd_knee
        Kp[5, 5] = Kp_ankle
        Kd[5, 5] = Kd_ankle

        self.pdController = PDController.PDController(model, Kp, Kd)

    def initilzie(self):
        count = 0
        self.u = []
        self.K = []
        A_ = []
        b_ = []
        J = 0
        self.max_steps = self._runner.get_length()
        P = self._runner._data["P"]
        start = self._runner.get_start()
        q = np.array([q[0] for q in start])
        qd = np.zeros(self.rbdl_model.qdot_size)
        y = np.concatenate((q, qd))
        tau = []

        while count < self._runner.get_length():
            self._runner.step()
            self.K.append(self._runner.K)
            u_raw = np.array(self._runner.ddx)
            x_raw = np.array(self._runner.x)
            xd_raw = np.array(self._runner.dx)
            tau.append(u_raw)
            u = np.array([q[0] for q in u_raw])
            x = np.array([q[0] for q in x_raw])
            xd = np.array([q[0] for q in xd_raw])
            y = np.concatenate((x, xd))
            #y2 = Model.runge_integrator(self.rbdl_model, y, 0.01, u)
            A, b = Model.finite_differences(self.rbdl_model, y, u, h=0.01)
            J += np.dot(np.dot(y.reshape((-1, 1)).T, (P[count])), y.reshape((-1, 1)))
            A_.append(A)
            b_.append(b)
            count += 1

        self.A = A_
        self.b = b_
        self.J = J

    def run_iLQR(self):
        A = self.A
        b = self.b
        eps = 1.0
        J = self.J
        print(J)
        while eps > 0.001:

            P, K = self.back_pass(A, b)
            A, b, J_, tau = self.foward_pass(K, P)
            eps = abs(J-J_)/J
            J = J_

        return K, tau, J_

    def foward_pass(self, K, P):
        count = 0
        h = 0.01
        A_ = []
        b_ = []
        x = self._runner.get_start()
        v0 = np.zeros(len(x)).reshape((-1, 1))
        J = 0
        y = np.concatenate((x, v0))
        tau = []

        while count < self.max_steps:
            # add ut here
            u = K[count].dot(np.vstack((self.expData[:, count].reshape((-1,1)), v0)) - y).flatten()
            # u = np.zeros(self.rbdl_model.qdot_size)
            y = Model.runge_integrator(self.rbdl_model, y.flatten(), h, u)
            A, b = Model.finite_differences(self.rbdl_model, y, u)
            A_.append(A)
            b_.append(b)
            J += np.dot(np.dot(y.reshape((-1, 1)).T, (P[count])), y.reshape((-1, 1)))
            tau.append(u)
            y = np.array([ [q] for q in y])
            count += 1

        return A_, b_, J, tau


    def back_pass(self, A, b):
        # need to get cost
        expSigma = self._runner._data["expSigma"]
        ric = solve_riccati_mat(expSigma, A, b)
        P = ric["P"]
        K = ric["K"]
        return P, K

    def calc_tau(self, q=None, qd=None, qdd=None, other=None):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        # t = time.process_time()
        # self.K2_, self.tau_, self.J_ = self.run_iLQR()
        # elapsed_time = time.process_time() - t
        # print(elapsed_time)

        aq = np.zeros(7)
        if self.step == int(other[0]):
            v0 = np.zeros(len(self._x)).reshape((-1, 1))
            x_ = np.append(self._x, self._dx).reshape((-1, 1))
            #x_ = np.append(q[0:6], qd[0:6]).reshape((-1, 1))

            #K = np.append(np.eye(6) * self._kp, np.eye(6) * self._kc, 1)
            #self.u = K.dot(np.vstack((self.expData[:, self.step].reshape((-1, 1)), v0)) - x_)
            self.u = self.K2[self.step].dot(np.vstack((self.expData[:, self.step].reshape((-1, 1)), v0)) - x_)

            self._dx = self._dx + self.u * 0.01
            self._x = self._x + self._dx * 0.01
            self.step += 1
            self.u = np.append(self.u, [0.0])


        e = np.append(self._x, [0.0]) - self._model.q
        ed = np.append(self._dx, [0.0]) - self._model.qd
        aq = self.pdController.calc_tau(e, ed)
        self.tau =  self.u #self._model.calculate_dynamics(qdd)
        return self.tau


def solve_riccati_mat(expSigma, A=None, B=None, dt=0.01, reg=1e-5):
    ric = {}
    size = expSigma[0].shape[0]
    if A is None:
        Ad = np.kron([[0, 1],[0, 0]], np.eye(size))*dt + np.eye(2*size)
    else:
        Ad = A
    if B is None:
        Bd = np.kron([[0], [1]], np.eye(size)) * dt
    else:
        Bd = B

    Q = np.zeros((size*2, size*2))
    R = np.eye(size)*reg
    P = [np.zeros((size*2, size*2))] * len(expSigma)
    P[-1][:size, :size] = np.linalg.pinv(expSigma[-1])
    K = [np.zeros((size*2, size*2))] * len(expSigma)
    for ii in range(len(expSigma)-2, -1, -1):
        Q[:size, :size] = np.linalg.pinv(expSigma[ii])
        B = P[ii + 1].dot(Bd[ii])
        C = np.linalg.pinv(np.dot(Bd[ii].T.dot(P[ii + 1]), Bd[ii]) + R)
        D = Bd[ii].T.dot(P[ii + 1])
        F = np.dot(np.dot(Ad[ii].T, B.dot(C).dot(D) - P[ii + 1]), Ad[ii])
        P[ii] = Q - F

    size = expSigma[0].shape[0]
    for i in range(len(expSigma)):
        v = np.linalg.inv(np.dot(np.dot(Bd[i].T, P[i]), Bd[i]) + R)
        K[i] = np.dot(np.dot(v.dot(Bd[i].T), P[i]), Ad[i])

    ric["Ad"] = Ad
    ric["Bd"] = Bd
    ric["R"] = R
    ric["P"] = P
    ric["K"] = K
    return ric