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
    env.render("human")

    stime = 20000
    gap = int(20000/300)

    rollout = np.zeros([9, stime])
    
    class K:
        def __init__(self):
            self.i = 0
        def __call__(self):
            self.i += 1
            return self.i
    k = K()


    rollout[1, int(k()*gap):]  +=  np.pi*0.0
    rollout[6, int(k()*gap):]  +=  np.pi*0.4
    rollout[7, int(k()*gap):]  +=  np.pi*0.3
    rollout[3, int(k()*gap):]  -=  np.pi*0.2
    rollout[5, int(k()*gap):]  -=  np.pi*0.45
    rollout[8, int(k()*gap):]  +=  np.pi*0.05
    for t in np.linspace(k()*gap, (k()+2)*gap, 35*gap):
        rollout[7, int(t):]  -=  np.pi*0.27/(35*gap)
    for t in range(35): k()
    rollout[5, int(k()*gap):]  -=  np.pi*0.4

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

        time.sleep(1/200)

    
