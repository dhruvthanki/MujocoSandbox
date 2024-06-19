import numpy as np
import cvxpy as cp
import mujoco


class WholeBodyController:
    def __init__(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.data2 = mujoco.MjData(self.model)
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        self.ctrl_range = self.model.actuator_ctrlrange[:, 1]
        self.mass_matrix = np.zeros((self.model.nv, self.model.nv))
        self.jac_pos = np.zeros((3, self.model.nv))
        self.jac_pos_plus = np.zeros((3, self.model.nv))
        self.kp_pos = 10
        self.kd_pos = 1

        self.osc_qp = self._formulate_qp()

    def _formulate_qp(self) -> cp.Problem:
        self.pH = cp.Parameter((self.model.nv, self.model.nv), name='H')
        self.vddq = cp.Variable(self.model.nv, name='ddq')
        self.pC_terms = cp.Parameter(self.model.nv, name='C_terms')
        self.vu = cp.Variable(self.model.nv, name='u')

        constraints = [self.pH @ self.vddq + self.pC_terms == self.vu]
        constraints += [self.vu <= self.ctrl_range, self.vu >= -self.ctrl_range]

        self.pSitePos = cp.Parameter(3, name='site_pos')
        self.pSitePosDes = cp.Parameter(3, name='site_pos_des')
        self.pSiteVelDes = cp.Parameter(3, name='site_vel_des')
        self.pSiteAccDes = cp.Parameter(3, name='site_acc_des')

        self.pJacPos = cp.Parameter((3, self.model.nv), name='JacPos')
        self.pJacPosPlus = cp.Parameter((3, self.model.nv), name='JacPosPlus')
        self.jac_pos_dot = (self.pJacPosPlus - self.pJacPos) / self.model.opt.timestep
        self.pdq = cp.Parameter(self.model.nv, name='dq')

        site_pos_ddot_des = self.pSiteAccDes + self.kd_pos * (self.pSiteVelDes - self.pJacPos @ self.pdq) + self.kp_pos * (self.pSitePosDes - self.pSitePos)
        site_pos_ddot = self.pJacPos @ self.vddq + self.jac_pos_dot @ self.pdq

        position_cost = cp.sum_squares(site_pos_ddot_des - site_pos_ddot)
        control_effort_cost = cp.sum_squares(self.vu)
        total_cost = position_cost + control_effort_cost

        objective = cp.Minimize(total_cost)

        return cp.Problem(objective, constraints)

    def compute_joint_torques(self, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
        self.data.qpos = qpos
        self.data.qvel = qvel
        mujoco.mj_forward(self.model, self.data)

        self.data2.qpos = self.data.qpos
        self.data2.qvel = self.data.qvel
        mujoco.mj_integratePos(self.model, self.data2.qpos, self.data2.qvel, self.model.opt.timestep)
        mujoco.mj_forward(self.model, self.data2)

        mujoco.mj_fullM(self.model, self.mass_matrix, self.data.qM)
        self.pH.value = self.mass_matrix
        self.pC_terms.value = self.data.qfrc_bias

        self.pSitePos.value = self.data.site_xpos[self.end_effector_id]
        self.pSitePosDes.value = np.array([0.5, 0.5, 0.5])
        self.pSiteVelDes.value = np.zeros(3)
        self.pSiteAccDes.value = np.zeros(3)

        mujoco.mj_jacSite(self.model, self.data, self.jac_pos, None, self.end_effector_id)
        self.pJacPos.value = self.jac_pos
        mujoco.mj_jacSite(self.model, self.data2, self.jac_pos_plus, None, self.end_effector_id)
        self.pJacPosPlus.value = self.jac_pos_plus
        self.pdq.value = self.data.qvel

        self.osc_qp.solve(warm_start=True, solver=cp.ECOS, verbose=False)

        return self.vu.value.squeeze()
