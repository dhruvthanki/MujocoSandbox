import numpy as np
import mujoco

class RobotDynamics:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.mass_matrix = np.zeros((self.model.nv, self.model.nv))
        self.ctrl_range = self.model.actuator_ctrlrange[:, 1]
        # self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        # self.jac_pos = np.zeros((3, self.model.nv))
        # self.data2 = mujoco.MjData(self.model)
        # self.jac_pos_plus = np.zeros((3, self.model.nv))

    def forward(self, qpos, qvel):
        self.data.qpos = qpos
        self.data.qvel = qvel

        mujoco.mj_forward(self.model, self.data)

    def get_mass_matrix(self):
        mujoco.mj_fullM(self.model, self.mass_matrix, self.data.qM)
        return self.mass_matrix
    
    def get_coriolis_terms(self):
        return self.data.qfrc_bias
    
    def get_nv(self):
        return self.model.nv
    
    def get_joint_torque_limits(self):
        return self.ctrl_range
    
    def get_keyframe(self, name):
        return self.model.keyframe(name)