from pybullet_envs.scene_abstract import SingleRobotEmptyScene
from pybullet_envs.env_bases import MJCFBaseBulletEnv
import numpy as np
import pybullet
from .realcomp_robot import Kuka

import sys

#import kukacomp.data as data
#bullet_client.setAdditionalSearchPath(data.getDataPath())

def DefaultRewardFunc(contact_dict, state):
    
    finger_touch = np.sum([ len([contact for contact in contacts 
        if not "table" in contact]) for part, contacts 
        in contact_dict.items() if "finger" in part ])

    return finger_touch

class REALCompEnv(MJCFBaseBulletEnv):
    def __init__(self, render=False):

        self.robot = Kuka()
        MJCFBaseBulletEnv.__init__(self, self.robot, render)
        
        self._cam_dist = 1.2
        self._cam_yaw = 30
        self._cam_roll = 0
        self._cam_pitch = -30
        self._render_width = 320
        self._render_height = 240
        self._cam_pos = [0,0,.4]
        self.setCamera()
        self.eyes = {}

        self.reward_func = DefaultRewardFunc
   
    def setCamera(self):
        self.envCamera = EnvCamera(
                distance=self._cam_dist, 
                yaw=self._cam_yaw,
                pitch=self._cam_pitch, 
                roll=self._cam_roll, 
                pos=self._cam_pos,
                width=self._render_width,
                height=self._render_height)
    
    def setEye(self, name):
        pos = self.robot.robot_position.copy()
        pos[2] = 1.4
        cam = EyeCamera(pos, [0, 0, 0])
        self.eyes[name] = cam

    def create_single_player_scene(self, bullet_client):
        return SingleRobotEmptyScene(bullet_client, gravity=9.81, 
                timestep=0.005, frame_skip=1)
    
    def reset(self):

        super(REALCompEnv, self).reset()
        self._p.setGravity(0.,0.,-9.81)
        self.camera._p = self._p
        for name in self.eyes.keys():
           self.eyes[name]._p = self._p
        
        self._p.resetDebugVisualizerCamera(
                self._cam_dist, self._cam_yaw, 
                self._cam_pitch, self._cam_pos)

    def render(self, mode='human', close=False):
            if mode == "human":
                    self.isRender = True
            if mode != "rgb_array":
                    return np.array([])

            rgb_array = self.envCamera.render(self._p)
            return rgb_array

    def step(self, a):
        assert(not self.scene.multiplayer)
        
        self.robot.apply_action(a)
        self.scene.global_step()
        state = self.robot.calc_state()
        contacts = self.robot.get_contacts()
        reward = self.reward_func(contacts, state)

        done = False
        info = {"contacts": contacts}

        return state, reward, done, info

class EnvCamera:

    def __init__(self, distance, yaw, pitch, roll, pos, 
            fov=80, width=320, height=240):
        
        self.dist = distance
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.pos = pos
        self.fov = fov
        self.render_width = width
        self.render_height = height

    def render(self, bullet_client = None):
        
        if bullet_client is None:
            bullet_client = self._p

        view_matrix = bullet_client.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition = self.pos,
                distance=self.dist,
                yaw=self.yaw,
                pitch=self.pitch,
                roll=self.roll,
                upAxisIndex=2)
        proj_matrix = bullet_client.computeProjectionMatrixFOV(
                fov=self.fov, aspect=float(self.render_width)/self.render_height,
                nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = bullet_client.getCameraImage(
                width=self.render_width, height=self.render_height,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix,
                renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
                )

        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]

        return rgb_array
            
class EyeCamera:

    def __init__(self, eyePosition, targetPosition,
            fov=80, width=320, height=240):
        
        self.eyePosition = eyePosition
        self.targetPosition = targetPosition
        self.upVector = [0, 0, 1]
        self.fov = fov
        self.render_width = width
        self.render_height = height
        self._p = None

    def render(self, targetPosition, bullet_client = None):
        
        if bullet_client is None:
            bullet_client = self._p

        self.targetPosition = targetPosition

        view_matrix = bullet_client.computeViewMatrix(
                cameraEyePosition = self.eyePosition,
                cameraTargetPosition = self.targetPosition,
                cameraUpVector=self.upVector)

        proj_matrix = bullet_client.computeProjectionMatrixFOV(
                fov=self.fov, aspect=float(self.render_width)/self.render_height,
                nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = bullet_client.getCameraImage(
                width=self.render_width, height=self.render_height,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix,
                renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
                )

        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]

        return rgb_array
            



