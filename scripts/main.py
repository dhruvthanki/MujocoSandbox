import mujoco
import mujoco.viewer
import keyboard
from overrides import overrides

from wholeBodyController import WholeBodyController
from PeriodicExecutor import PeriodicExecutor

class Simulation(PeriodicExecutor):
    def __init__(self, model_path, pause_at_start, duration=30):
        super().__init__(frequency=100)
        self.is_paused = True
        self.duration = duration
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.data.qpos = self.model.keyframe('home').qpos
        mujoco.mj_forward(self.model, self.data)
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
    
    @overrides
    def periodic_function(self):
        if self.viewer.is_running() and not self.is_paused:
            try:
                # self.data.ctrl = -0.1 * self.data.qvel + 100 * (self.model.keyframe('home').qpos - self.data.qpos)
                self.data.ctrl = self.controller.compute_joint_torques(self.data.qpos, self.data.qvel)
            except Exception as e:
                print(e)
            mujoco.mj_step(self.model, self.data)
            
            # print(self.data.site_xpos[self.controller.dynamic_model.end_effector_id])
            # print(self.controller.dynamic_model.get_end_effector_ori())
            # print(self.data.qpos)

            with self.viewer.lock():
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.data.time % 2)

            self.viewer.sync()
        elif not self.viewer.is_running():
            self.stop()
    
    @overrides
    def stop(self):
        super().stop()
        if self.viewer is not None:
            self.viewer.close()
            print("Viewer closed.")
        # keyboard.unhook_all_hotkeys()

    def __del__(self):
        self.join()

if __name__ == "__main__":
    pause_at_start = True
    # model_path = 'submodules/mujoco_menagerie/kuka_iiwa_14/scene.xml'
    # model_path = 'models/DoublePendulum.xml'
    model_path = 'models/ur5/ur5.xml'
    sim = Simulation(model_path, pause_at_start)
    sim.start()
    try:
        sim.join()
    except KeyboardInterrupt:
        pass
    finally:
        del sim
        print("Simulation deleted.")
