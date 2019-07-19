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

scores = {}

def add_scores(challenge, score):
    if challenge in scores.keys():
        scores[challenge] += [score]
    else:
        scores[challenge] = [score]

def report_score():
    print("*****************")
    total_score = 0
    challenges = ['2D','2.5D','3D']
    for key in challenges:
        if key in scores.keys():
            results = scores[key]
            formatted_results = ", ".join(["{:.4f}".format(r) for r in results])
            challenge_score = np.mean(results)
        else:
            results = []
            formatted_results = "None"
            challenge_score = 0

        print("Challenge {} - {:.4f}".format(key, challenge_score))
        print("Goals: {}".format(formatted_results))
        total_score += challenge_score
    total_score /= len(challenges)
    print("Overall Score: {:.4f}".format(total_score))  
    print("*****************")


def demo_run(extrinsic_trials=350):

    env = gym.make('REALComp-v0')
    controller = Controller(env.action_space)
    
    # change length of simulation for testing purposes
    env.intrinsic_timesteps = 1e7 #default = 1e7
    env.extrinsic_timesteps = 2e3 #default = 2e3

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
    totalScore = 0
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

        add_scores(*env.evaluateGoal())
        print("Current score:")
        report_score()
       
    print("Final score:")
    report_score()

if __name__=="__main__":
    demo_run()
