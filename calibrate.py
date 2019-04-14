import matplotlib
matplotlib.use("Agg")

import gym, os, glob 
from OpenGL import GLU
import numpy as np
import matplotlib.pyplot as plt
import DMp
from DMp.bbo_pdmp import BBO, rew_softmax
import realcomp 
from PIL import Image
import time

class Simulation:
    def __init__(self, rollout, env, plot=False):
        """
        :param rollout: A single rollout (n_joints x timesteps) from which joint commands are taken
        :param plot: if the simulation is rendered on a window
        """
        self.t = 0
        self.rollout = rollout  
        self.plot = plot
        self.env = env
        self.env.reset()

        
    def __call__(self):    

        # we control only few joints
        ctrl_joints = self.rollout[:, self.t]
        action = np.zeros(9)
        
        action[0]   =  np.pi*0.0 + ctrl_joints[0]
        action[1]   =  np.pi*0.2 + ctrl_joints[1]
        action[2]   =  np.pi*0.0 + ctrl_joints[2] 
        action[3]   = -np.pi*0.2 + ctrl_joints[3]
        action[4:7] =  np.pi*0.0 + ctrl_joints[4:7] 
        action[7:] = ctrl_joints[7:]*np.pi
        
        # do the movement
        state, r, done, info_ = self.env.step(action)
    
        if len(info_["contacts"]) > 0:
            print(info_["contacts"])

        self.t += 1  
        return r

if __name__ == "__main__":
    
    env = gym.make("REALComp-v0")
    env._render_width = 640
    env._render_height = 480
    env._cam_yaw = 180
    env.setCamera()

    env.reward_func = lambda x, y: 0
    env.robot.used_objects = ["table", "tomato", "mustard", "orange"]
    env.robot.object_poses["mustard"][2] = 1 
    env.render("human")

    stime = 200
    rollout = np.zeros([9, stime])
    rollout[0, int(stime*1/10):] +=  np.pi*0.18
    rollout[1, int(stime*3/10):] +=  np.pi*0.06
    rollout[6, int(stime*4/10):] +=  np.pi*0.4
    rollout[7, int(stime*5/10):] +=  np.pi*0.3
    rollout[5, int(stime*6/10):] +=  np.pi*0.4
    rollout[7, int(stime*7/10):] -=  np.pi*0.35
    rollout[8, int(stime*7/10):] +=  np.pi*0.05
    rollout[5, int(stime*8/10):] -=  np.pi*0.4

    sim = Simulation(rollout, env, plot=False)


    for t in range(len(rollout.T)): 
        time.sleep(1/60)
        sim()

    
