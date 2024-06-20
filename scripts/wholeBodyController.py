import numpy as np
import cvxpy as cp

from RobotDynamics import RobotDynamics
from QuatUtils import shortest_error_quaternion

class WholeBodyController:
    def __init__(self, model_path: str):
        self.dynamic_model = RobotDynamics(model_path)
        self.nv = self.dynamic_model.get_nv()
        self.nq = self.dynamic_model.get_nv()

        self.kp_pos = 50
        self.kd_pos = 10

        self.kp_ori = 700
        self.kd_ori = 50

        self.kp_joint = 500
        self.kd_joint = (np.sqrt(self.kp_joint)/2) + 0

        self.osc_qp = self._formulate_qp()

    def _formulate_qp(self) -> cp.Problem:
        # Constraints
        self.pH = cp.Parameter((self.nv, self.nv), name='H')
        self.vddq = cp.Variable(self.nv, name='ddq')
        self.pC_terms = cp.Parameter(self.nv, name='C_terms')
        self.vu = cp.Variable(self.nv, name='u')

        constraints = [self.pH @ self.vddq + self.pC_terms == self.vu]
        constraints += [self.vu <= self.dynamic_model.get_joint_torque_limits(), self.vu >= -self.dynamic_model.get_joint_torque_limits()]

        total_cost = 0

        # Joint position cost
        self.pq = cp.Parameter(self.nq, name='q')
        self.pdq = cp.Parameter(self.nv, name='dq')

        joint_pos_ddot_des = - self.kd_joint * self.pdq + self.kp_joint * (self.dynamic_model.get_keyframe('home').qpos - self.pq)
        joint_position_cost = cp.sum_squares(joint_pos_ddot_des - self.vddq)
        # total_cost += 0.001*joint_position_cost

        # End effector position
        self.pSitePos = cp.Parameter(3, name='site_pos')
        self.pSitePosDes = cp.Parameter(3, name='site_pos_des')
        self.pSiteVelDes = cp.Parameter(3, name='site_vel_des')
        self.pSiteAccDes = cp.Parameter(3, name='site_acc_des')

        self.pJacPos = cp.Parameter((3, self.nv), name='JacPos')
        self.pJacPosPlus = cp.Parameter((3, self.nv), name='JacPosPlus')
        self.jac_pos_dot = (self.pJacPosPlus - self.pJacPos) / self.dynamic_model.get_time_step()

        site_pos_ddot_des = self.pSiteAccDes + self.kd_pos * (self.pSiteVelDes - self.pJacPos @ self.pdq) + self.kp_pos * (self.pSitePosDes - self.pSitePos)
        site_pos_ddot = self.pJacPos @ self.vddq + self.jac_pos_dot @ self.pdq
        position_cost = cp.sum_squares(site_pos_ddot_des - site_pos_ddot)
        total_cost += position_cost

        # End effector orientation
        self.pSiteOriDiff = cp.Parameter(3, name='site_ori')
        self.pSiteAngularVelDes = cp.Parameter(3, name='site_ang_vel_des')
        self.pSiteAngularAccDes = cp.Parameter(3, name='site_ang_acc_des')

        self.pJacOri = cp.Parameter((3, self.nv), name='JacOri')
        self.pJacOriPlus = cp.Parameter((3, self.nv), name='JacOriPlus')
        self.jac_ori_dot = (self.pJacOriPlus - self.pJacOri) / self.dynamic_model.get_time_step()

        site_ori_ddot_des = self.pSiteAngularAccDes + self.kd_ori * (self.pSiteAngularVelDes - self.pJacOri @ self.pdq) + self.kp_ori * self.pSiteOriDiff
        site_ori_ddot = self.pJacOri @ self.vddq + self.jac_ori_dot @ self.pdq
        orientation_cost = cp.sum_squares(site_ori_ddot_des - site_ori_ddot)
        total_cost += 5*orientation_cost

        # Control effort cost
        control_effort_cost = cp.sum_squares(self.vu)
        # total_cost += control_effort_cost

        objective = cp.Minimize(total_cost)

        return cp.Problem(objective, constraints)

    def compute_joint_torques(self, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
        self.dynamic_model.forward(qpos, qvel)

        self.pH.value = self.dynamic_model.get_mass_matrix()
        self.pC_terms.value = self.dynamic_model.get_coriolis_terms()
        self.pq.value = qpos
        self.pdq.value = qvel

        self.pSitePos.value = self.dynamic_model.get_end_effector_pos()
        self.pSitePosDes.value = np.array([0.10045471, 0.1337827,  0.79955219])
        self.pSiteVelDes.value = np.zeros(3)
        self.pSiteAccDes.value = np.zeros(3)

        self.pSiteOriDiff.value = shortest_error_quaternion(np.array([0.0, 0.0, 0.0, 1.0]), self.dynamic_model.get_end_effector_ori())
        self.pSiteAngularVelDes.value = np.zeros(3)
        self.pSiteAngularAccDes.value = np.zeros(3)

        self.pJacPos.value, self.pJacOri.value = self.dynamic_model.get_site_jacobian()
        self.pJacPosPlus.value, self.pJacOriPlus.value = self.dynamic_model.get_site_jacobian_plus()

        self.osc_qp.solve(warm_start=True, solver=cp.ECOS, verbose=False)

        print(self.osc_qp.status)
        return self.vu.value.squeeze()
