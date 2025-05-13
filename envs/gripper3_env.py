import time
import dm_env
import numpy as np
import collections
import cv2
from robots.gripper3_robot.gripper3 import GRIPPER3

class Gripper3Env(object):
    def __init__(self):
        self.exo = GRIPPER3()
        self.joints_num = self.exo.servo_num
        # 视频采集初始化
        self.cam = cv2.VideoCapture('/dev/video2')
        if not self.cam.isOpened():
            self.get_logger().error('Cannot open camera')
        self.cam_name = "0" 

    def reset(self, sleep_time=2.0):
        time.sleep(sleep_time)
        self.exo.go_zero()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self._get_obs(),
        )
    
    def _get_obs(self):
        obs = {}
        obs["qpos"] = []
        obs["images"] = {}
        obs["qpos"] = self.get_qpos()  
        obs["images"][self.cam_name] = self.get_images()
    
        return obs

    def step(
        self,
        action,
        get_obs=True,
        sleep_time=0.0,
    ):
        assert len(action) == self.joints_num, \
            f"Action must be length {self.joints_num}"
        self.exo.servo_ctrl_callback(action)
        time.sleep(sleep_time)
        if get_obs:
            obs = self._get_obs()
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs,
        )

    def get_qpos(self):
        servo_pos = self.exo.get_current_joint_positions()
        qpos = np.array(servo_pos, dtype=np.float32) * 3.14 / 2048
        return qpos
    
    def get_images(self):
        ret, frame = self.cam.read()
        if ret:
            return frame
    
    def get_reward(self):
        return 0
    
def make_env(config):

    env = Gripper3Env(config["env_config_path"])
    env.set_reset_position(config["start_joint"])

    return env