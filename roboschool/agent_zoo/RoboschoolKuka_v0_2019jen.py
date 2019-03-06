import os.path, time, gym
from OpenGL import GLU
import numpy as np
import roboschool
import time
from PIL import Image





def demo_run():
   
    np.set_printoptions(formatter={"float": "{:4.2f}".format})

    actions = np.array([
        [0,    0,    0,     0,    0,    0,    0,   .8,    .8],   
        [0,    0,    0,  -1.0,  1.0,    0,    0,   .8,    .8],   
        [0,  0.29,   0,  -0.95,  1.0,   0,    0,   .8,    .8],   
        [0,  0.29,   0,  -0.95,  1.0,   0,    0,   .25,   .3]])*np.pi/2.0

    env = gym.make("RoboschoolKuka-v0")
    env.unwrapped.EYE_W=32
    env.unwrapped.EYE_H=32


    for k in range(10):
        obs = env.reset()    
        for t in range(1000):
            if         t <  5:
                a = actions[0].copy()
            elif  5 < t <=20:
                a = actions[1].copy()
            elif 20 < t <=40:
                a = actions[2].copy()
            elif 40 < t <=600:
              a = actions[2].copy()
              a[7] = np.maximum(0.25,a[7] - (t-40)*0.01)
              a[8] = np.maximum(0.3,a[8] - (t-40)*0.001)
            elif t >600:
              a[1] = np.maximum(0.,a[1] - (t-60)*0.01)
            
            state, r, done, info_ = env.step(a)
           
            still_open = env.render("human")
            contacts, rgb_eye = (info_["contacts"], info_["rgb_eye"])

            eye_im = Image.fromarray(rgb_eye)          
            eye_im.save("eye_{:04d}.jpg".format(t))

            if len(contacts)>0:
                for body_part, conts in contacts.items():
                    print("{} : {}".format(body_part, conts))
            else:
                print("--")
           
            #if not still_open: return
           
             
if __name__=="__main__":

    demo_run()
