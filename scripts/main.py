import time

import mujoco
import mujoco.viewer
import keyboard

from wholeBodyController import WholeBodyController

class Simulation:
    def __init__(self, model_path, pause_at_start, duration=30):
        self.is_paused = True
        self.duration = duration
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        if pause_at_start:
            keyboard.add_hotkey('space', self.toggle_pause)
        else:
            self.toggle_pause()
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.controller = WholeBodyController(model_path)
    
    def toggle_pause(self):
        self.is_paused = not self.is_paused
        if self.is_paused:
            print("Simulation paused.")
        else:
            print("Simulation resumed.")

    def run(self):
        start = time.time()
        while self.viewer.is_running() and time.time() - start < self.duration:
            step_start = time.time()

            if not self.is_paused:
                self.data.ctrl = self.controller.compute_joint_torques(self.data.qpos, self.data.qvel)
                mujoco.mj_step(self.model, self.data)

            with self.viewer.lock():
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.data.time % 2)

            self.viewer.sync()

            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    def __del__(self):
        if self.viewer is not None:
            self.viewer.close()
            print("Viewer closed.")

if __name__ == "__main__":
    pause_at_start = False
    sim = Simulation('submodules/mujoco_menagerie/kuka_iiwa_14/scene.xml', pause_at_start)
    sim.run()
