import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

class RobotDynamics:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.mass_matrix = np.zeros((self.model.nv, self.model.nv))
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "EndEffector")
        self.jac_pos = np.zeros((3, self.model.nv))
        self.data2 = mujoco.MjData(self.model)
        self.jac_pos_plus = np.zeros((3, self.model.nv))
        self.jac_ori = np.zeros((3, self.model.nv))
        self.jac_ori_plus = np.zeros((3, self.model.nv))

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
        return self.model.actuator_ctrlrange[:,1]
    
    def get_keyframe(self, name):
        return self.model.keyframe(name)
    
    def get_end_effector_pos(self):
        return self.data.site_xpos[self.end_effector_id,:]
    
    def get_end_effector_ori(self):
        '''Returns the quaternion representation of the end effector orientation'''
        rotation = self.data.site_xmat[self.end_effector_id,:].reshape(3, 3)
        quat = R.from_matrix(rotation).as_quat()
        return quat
    
    def get_site_jacobian(self):
        '''Returns the position and orientation geometric jacobian of the end effector'''
        mujoco.mj_jacSite(self.model, self.data, self.jac_pos, self.jac_ori, self.end_effector_id)
        return self.jac_pos, self.jac_ori
    
    def get_site_jacobian_plus(self):
        '''Returns the position and orientation geometric jacobian of the end effector at the next time step'''
        self.data2.qpos = self.data.qpos
        self.data2.qvel = self.data.qvel
        mujoco.mj_integratePos(self.model, self.data2.qpos, self.data2.qvel, self.model.opt.timestep)
        mujoco.mj_forward(self.model, self.data2)
        mujoco.mj_jacSite(self.model, self.data2, self.jac_pos_plus, self.jac_ori_plus, self.end_effector_id)
        return self.jac_pos_plus, self.jac_ori_plus
    
    def get_time_step(self):
        return self.model.opt.timestep