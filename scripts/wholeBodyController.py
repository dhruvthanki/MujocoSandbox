import numpy as np
from pyquaternion import quaternion
import cvxpy as cp
import mujoco

class WholeBodyController:
    def __init__(self, model_path='submodules/mujoco_menagerie/unitree_g1/scene.xml'):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)