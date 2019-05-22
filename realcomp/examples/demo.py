from OpenGL import GLU
import numpy as np
import realcomp
from realcomp.envs.realcomp_env import Goal
import gym

class RandomPolicy:
    """
    A fake controller chosing random actions
    """
    def __init__(self, action_space):
        self.action_space = action_space
        self.action = np.zeros(action_space.shape[0])

    def step(self, observation, reward, done):
        self.action += 0.1*np.pi*np.random.randn(self.action_space.shape[0])
        return self.action

Controller = RandomPolicy

def demo_run(extrinsic_trials=10):

    env = gym.make('REALComp-v0')
    controller = Controller(env.action_space)
    
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
    print(dir())
    demo_run()
