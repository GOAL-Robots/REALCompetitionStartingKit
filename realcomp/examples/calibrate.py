#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import gym
import realcomp 
import time
import numpy as np

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

    stime = 200000
    rollout = np.zeros([9, stime])
    rollout[0, int(stime*1/10):] +=  np.pi*0.18
    rollout[1, int(stime*3/10):] +=  np.pi*0.06
    rollout[6, int(stime*4/10):] +=  np.pi*0.4
    rollout[7, int(stime*5/10):] +=  np.pi*0.3
    rollout[5, int(stime*6/10):] +=  np.pi*0.4
    rollout[7, int(stime*7/10):] -=  np.pi*0.35
    rollout[8, int(stime*7/10):] +=  np.pi*0.05
    rollout[5, int(stime*8/10):] -=  np.pi*0.4


    env.reset()
    for t in range(len(rollout.T)): 
        
        # we control only few joints
        ctrl_joints = rollout[:, t]
        action = np.zeros(9)
        
        action[0]   =  np.pi*0.0 + ctrl_joints[0]
        action[1]   =  np.pi*0.2 + ctrl_joints[1]
        action[2]   =  np.pi*0.0 + ctrl_joints[2] 
        action[3]   = -np.pi*0.2 + ctrl_joints[3]
        action[4:7] =  np.pi*0.0 + ctrl_joints[4:7] 
        action[7:] = ctrl_joints[7:]*np.pi
        
        # do the movement
        state, r, done, info_ = env.step(action)
    
        if len(info_["contacts"]) > 0:
            print(info_["contacts"])
        
        time.sleep(1/60)

    
