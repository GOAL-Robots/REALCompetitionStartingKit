#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import pybullet
import gym
import numpy as np
from realcomp import envs
import time

import pyglet, pyglet.window as pw, pyglet.window.key as pwk
from pyglet import gl


def main():

    env = gym.make("REALComp-v0")
    
    pi = RandomPolicy(env.action_space)
    p = PygletInteractiveWindow(env, 320, 240)

    env.render("rgb_array")

    for k in range(10):
        observation = env.reset()
        for t in range(40):
            time.sleep(1./1000.)
            a = pi.act()
            observation, reward, done, info = env.step(a)
            
            sensors_rng = range(env.robot.num_joints, (env.robot.num_joints + env.robot.num_touch_sensors))
            print(observation[sensors_rng])
            
            retina_start_idx = env.robot.num_joints + env.robot.num_touch_sensors
            w = env.robot.eye_width
            h = env.robot.eye_height
            retina = observation[retina_start_idx:].reshape(w, h, 3)
            p.imshow(env.get_retina())

class PygletInteractiveWindow(pw.Window):
    
    def __init__(self, env, width, height):

        pw.Window.__init__(self, width=width, height=height, vsync=False, resizable=True)
        self.theta = 0
        self.still_open = True

        @self.event
        def on_close():
            self.still_open = False

        @self.event
        def on_resize(width, height):
            self.win_w = width
            self.win_h = height

        self.keys = {}
        self.human_pause = False
        self.human_done = False

    def imshow(self, arr):

        H, W, C = arr.shape
        assert C==3
        image = pyglet.image.ImageData(W, H, 'RGB', arr.tobytes(), pitch=W*-3)
        self.clear()
        self.switch_to()
        self.dispatch_events()
        texture = image.get_texture()
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)
        texture.width  = W
        texture.height = H
        texture.blit(0, 0, width=self.win_w, height=self.win_h)
        self.flip()



class RandomPolicy:

    def __init__(self, action_space):
        self.action_space = action_space
        self.action = np.zeros(action_space.shape[0])
        self.action += -np.pi*0.5

    def act(self):
        self.action += 0.4*np.pi*np.random.randn(self.action_space.shape[0])
        return self.action

if __name__=="__main__":
    
    main()
