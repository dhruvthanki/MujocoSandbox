import numpy as np
import cvxpy as cp

from RobotDynamics import RobotDynamics

class WholeBodyController:
    def __init__(self, model_path: str):
        self.dynamic_model = RobotDynamics(model_path)
        self.nv = self.dynamic_model.get_nv()
        self.nq = self.dynamic_model.get_nv()

        # self.kp_pos = 50
        # self.kd_pos = 10

        self.kp_joint = 500
        self.kd_joint = (np.sqrt(self.kp_joint)/2)

        self.osc_qp = self._formulate_qp()

    def _formulate_qp(self) -> cp.Problem:
        self.pH = cp.Parameter((self.nv, self.nv), name='H')
        self.vddq = cp.Variable(self.nv, name='ddq')
        self.pC_terms = cp.Parameter(self.nv, name='C_terms')
        self.vu = cp.Variable(self.nv, name='u')

        constraints = [self.pH @ self.vddq + self.pC_terms == self.vu]
        constraints += [self.vu <= self.dynamic_model.get_joint_torque_limits(), self.vu >= -self.dynamic_model.get_joint_torque_limits()]

        total_cost = 0

        self.pq = cp.Parameter(self.nq, name='q')
        self.pdq = cp.Parameter(self.nv, name='dq')

        joint_pos_ddot_des = -self.kd_joint * self.pdq + self.kp_joint * (self.dynamic_model.get_keyframe('home').qpos - self.pq)
        joint_position_cost = cp.sum_squares(joint_pos_ddot_des - self.vddq)
        total_cost += joint_position_cost

        # self.pSitePos = cp.Parameter(3, name='site_pos')
        # self.pSitePosDes = cp.Parameter(3, name='site_pos_des')
        # self.pSiteVelDes = cp.Parameter(3, name='site_vel_des')
        # self.pSiteAccDes = cp.Parameter(3, name='site_acc_des')

        # self.pJacPos = cp.Parameter((3, self.nv), name='JacPos')
        # self.pJacPosPlus = cp.Parameter((3, self.nv), name='JacPosPlus')
        # self.jac_pos_dot = (self.pJacPosPlus - self.pJacPos) / self.model.opt.timestep

        # site_pos_ddot_des = self.pSiteAccDes + self.kd_pos * (self.pSiteVelDes - self.pJacPos @ self.pdq) + self.kp_pos * (self.pSitePosDes - self.pSitePos)
        # site_pos_ddot = self.pJacPos @ self.vddq + self.jac_pos_dot @ self.pdq
        # position_cost = cp.sum_squares(site_pos_ddot_des - site_pos_ddot)
        # total_cost += position_cost

        # control_effort_cost = cp.sum_squares(self.vu)
        # total_cost += control_effort_cost

        objective = cp.Minimize(total_cost)

        return cp.Problem(objective, constraints)

    def compute_joint_torques(self, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
        self.dynamic_model.forward(qpos, qvel)

        self.pH.value = self.dynamic_model.get_mass_matrix()
        self.pC_terms.value = self.dynamic_model.get_coriolis_terms()
        self.pq.value = qpos
        self.pdq.value = qvel

        # self.pSitePos.value = self.data.site_xpos[self.end_effector_id]
        # self.pSitePosDes.value = np.array([6.63291267e-01, 1.61385684e-04, 2.62866404e-01])
        # self.pSiteVelDes.value = np.zeros(3)
        # self.pSiteAccDes.value = np.zeros(3)

        # mujoco.mj_jacSite(self.model, self.data, self.jac_pos, None, self.end_effector_id)
        # self.pJacPos.value = self.jac_pos

        # self.data2.qpos = self.data.qpos
        # self.data2.qvel = self.data.qvel
        # mujoco.mj_integratePos(self.model, self.data2.qpos, self.data2.qvel, self.model.opt.timestep)
        # mujoco.mj_forward(self.model, self.data2)
        # mujoco.mj_jacSite(self.model, self.data2, self.jac_pos_plus, None, self.end_effector_id)
        # self.pJacPosPlus.value = self.jac_pos_plus

        self.osc_qp.solve(warm_start=True, solver=cp.ECOS, verbose=False)

        return self.vu.value.squeeze()
