from OpenGL import GLU
import numpy as np
import realcomp
from realcomp.envs.realcomp_env import Goal
import gym

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
from my_controller import MyController

Controller = MyController

def demo_run(extrinsic_trials=10):

    env = gym.make('REALComp-v0')
    controller = Controller(env.action_space)
    
    env.intrinsic_timesteps = 2000
    env.extrinsic_timesteps = 10

    # render simulation on screen
    # env.render('human')
    
    # reset simulation
    observation = env.reset()  
    reward = 0 
    done = False

    # intrinsic phase
    print("Starting intrinsic phase...")
    while not done:
        
        # Call your controller to chose action 
        action = controller.step(observation, reward, done)
        
        # do action
        observation, reward, done, _ = env.step(action)
        
        # get frames for video making
        # rgb_array = env.render('rgb_array')
        
    # extrinsic phase
    print("Starting extrinsic phase...")
    for k in range(extrinsic_trials):
        
        # reset simulation
        observation = env.reset()  
        reward = 0 
        done = False    
        
        # set the extrinsic goal to pursue 
        env.set_goal()
        print("Starting extrinsic trial...")

        while not done:

            # Call your controller to chose action 
            action = controller.step(observation, reward, done)
            
            # do action
            observation, reward, done, _ = env.step(action)
            
            # get frames for video making
            # rgb_array = env.render('rgb_array')

if __name__=="__main__":
    demo_run()
