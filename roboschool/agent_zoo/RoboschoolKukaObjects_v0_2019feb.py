import os.path, time, gym
from OpenGL import GLU
import numpy as np
import roboschool
import time

def demo_run():
   

    env = gym.make("RoboschoolKuka-v1")
    env.unwrapped.set_eyeEnable(False)
    env.unwrapped.set_eyeShow(False)


    while True:
        obs = env.reset()   
        for t in range(100):
            a = np.zeros(9) 
            state, r, done, info_ = env.step(a)
            still_open = env.render("human")
        
             
if __name__=="__main__":

    demo_run()
