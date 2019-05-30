import numpy as np


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

MyController = RandomPolicy

