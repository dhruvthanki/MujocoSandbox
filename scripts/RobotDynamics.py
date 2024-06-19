import mujoco

class RobotDynamics:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.data2 = mujoco.MjData(self.model)
        self.end_effector_id = self.model.site_name2id('ee_site')