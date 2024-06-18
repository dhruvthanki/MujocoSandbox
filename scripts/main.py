import time

import mujoco
import mujoco.viewer
import keyboard

class Simulation:
    def __init__(self, model_path, duration=30):
        self.is_paused = True
        self.duration = duration
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        keyboard.add_hotkey('space', self.toggle_pause)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
    
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
    sim = Simulation('submodules/mujoco_menagerie/unitree_g1/scene.xml')
    sim.run()
