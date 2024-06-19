import numpy as np
from pyquaternion import quaternion
import cvxpy as cp
import mujoco

class WholeBodyController:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

    def compute_joint_torques(self, qpos, qvel):
        # Compute the joint torques to apply to the robot
        # to achieve the desired accelerations.
        # Here we just return zeros, which means the robot
        # will not move.
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        mujoco.mj_forward(self.model, self.data)

        return np.zeros(self.model.nv)