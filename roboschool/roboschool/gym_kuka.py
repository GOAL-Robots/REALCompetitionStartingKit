from roboschool.gym_urdf_robot_env import RoboschoolUrdfEnv
from roboschool.scene_abstract import SingleRobotEmptyScene
from roboschool.scene_abstract import cpp_household


import numpy as np
import gym
import copy
import sys
import os
import pyglet, pyglet.window as pw, pyglet.window.key as pwk
from pyglet import gl

#
# This opens a third-party window (not test window), shows rendered chase camera, allows to control humanoid
# using keyboard (in a different way)
#

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

    def on_key_press(self, key, modifiers):
        self.keys[key] = +1
        if key==pwk.ESCAPE: self.still_open = False

    def on_key_release(self, key, modifiers):
        self.keys[key] = 0

    def each_frame(self):
        self.theta += 0.05 * (self.keys.get(pwk.LEFT, 0) - self.keys.get(pwk.RIGHT, 0))

#------------------------------------------------------------------------------

def DefaultRewardFunc(contact_dict, state):
    
    finger_touch = np.sum([ len([contact for contact in contacts 
        if not "table" in contact]) for part, contacts 
        in contact_dict.items() if "finger" in part ])

    return finger_touch

class RoboschoolKuka(RoboschoolUrdfEnv):
    
    EYE_W = 200
    EYE_H = 200

    
    def create_single_player_scene(self):
        return SingleRobotEmptyScene(gravity=9.8, 
                timestep=0.01, frame_skip=1)

    def __init__(self):
        
        self.action_dim = 9
        self.obs_dim = 12*3

        super(RoboschoolKuka, self).__init__(
            "kuka_gripper_description/urdf/kuka_gripper.urdf",
            "pelvis",
            action_dim=self.action_dim, obs_dim=self.obs_dim,
            fixed_base=False,
            self_collision=True)

        fixed_base=False
        self_collision=True

        low = -0.75*np.pi*np.ones([self.action_dim])
        low[-2:] = 0
        high = 0.75*np.pi*np.ones([self.action_dim])

        self.action_space = gym.spaces.Box(low, high)
        high = np.inf*np.ones([self.obs_dim])
        self.observation_space = gym.spaces.Box(-high, high)

        self.rendered_rgb_eye = np.zeros([self.EYE_H, self.EYE_W, 3], dtype=np.uint8)

        self.reward_func = DefaultRewardFunc
        self.EYE_ENABLE = False
        self.EYE_SHOW = False
        
        
    def get_contacts(self):
        
        contact_dict = {}

        for part in self.urdf.parts:
            name = part.name
            contacts = [contact for contact 
                    in part.contact_list()
                    if not contact.name in self.robot_parts_names]
            
            if len(contacts)>0:
                contact_dict[name] = [contact.name 
                        for contact in contacts]
        return contact_dict
   
    def _reset(self):
        s = super(RoboschoolKuka, self)._reset()
        self.robot_parts_names = [part.name for part 
                in self.cpp_robot.parts]
        if self.EYE_ENABLE:
            self.eye = self.scene.cpp_world.new_camera_free_float
            (self.EYE_W, self.EYE_H, "eye")
        return s

    def _render(self, mode, close):
        render_res = super(RoboschoolKuka, self)._render(mode, close)
        
        if(self.EYE_ENABLE):
        
            self.eye_adjust() 
            
            # render(render_depth, render_labeling, print_timing))
            rgb_eye, _, _, _, _ = self.eye.render(False, False, False) 
            self.rendered_rgb_eye = np.fromstring(rgb_eye, dtype=np.uint8).reshape( 
                    (self.EYE_H,self.EYE_W,3) )
        
        return render_res

    def robot_specific_reset(self):
         
        pose_robot = cpp_household.Pose()
        pose_robot.set_xyz(-0.8, 0, 0)
        self.cpp_robot.set_pose_and_speed(pose_robot, 0,0,0)

        # add table
        pose_table = cpp_household.Pose()
        self.urdf_table  = self.scene.cpp_world.load_urdf(
            os.path.join(os.path.dirname(__file__), "models_robot",
                "kuka_gripper_description/urdf/table.urdf"),
            pose_table, True, True)
        
        # add cube
        pose_cube = cpp_household.Pose()
        pose_cube.set_xyz(0.0, 0, 0.482)
        self.urdf_cube  = self.scene.cpp_world.load_urdf(
            os.path.join(os.path.dirname(__file__), "models_robot",
                "kuka_gripper_description/urdf/cube.urdf"),
            pose_cube, False, True)
                
    def apply_action(self, a):

        assert(len(a) == 9)

        kp = 0.1
        kd = 1.0
        vel = 400

        a[:-2]  = np.maximum(-np.pi*0.5, np.minimum(np.pi*0.5, a[:-2]))
        a[-2:]  = np.maximum(          0, np.minimum(np.pi*0.5, a[-2:]))
        a[-1]  = np.maximum(           0, np.minimum(2*a[-2],    a[-1]))
  
        for i,j in enumerate(a[:-2]):
            self.jdict["lbr_iiwa_joint_%d"%(i+1)].set_servo_target(
                    j, kp, kd, vel)
        self.jdict["base_to_finger00_joint"].set_servo_target(
                a[-2],  kp, kd, vel)
        self.jdict["base_to_finger10_joint"].set_servo_target(
                 a[-2],  kp, kd, vel)
        self.jdict["finger00_to_finger01_joint"].set_servo_target(
                 -a[-1],  kp, kd, vel)
        self.jdict["finger10_to_finger11_joint"].set_servo_target(
                 -a[-1],  kp, kd, 0.01*vel)
    
    def _step(self, a):
        assert(not self.scene.multiplayer)
        
        self.apply_action(a)
        
        self.scene.global_step()

        state = self.calc_state()
        self.rewards = [ self.get_contacts()]

        reward = 0

        contact_dict = self.get_contacts()
        reward = self.reward_func(contact_dict, state)
        
        if self.EYE_ENABLE:
            self.eye_render()

        done = False
        info = {"contacts":self.get_contacts(), "rgb_eye": self.rendered_rgb_eye}

        return state, reward, done, info
    
    def calc_state(self):
        state = np.hstack([part.pose().xyz() for part in  self.cpp_robot.parts])
        state = np.hstack([state, self.urdf_cube.root_part.pose().xyz()])
        return state 

    def camera_adjust(self): 
        x, y, z = self.cpp_robot.root_part.pose().xyz()
        y += 0.5
        z += 0.3
        self.camera.move_and_look_at(.7, -.7, .8, x, y, z)
    
    def eye_render(self):
        self.eye_adjust() 
        rgb_eye, _, _, _, _ = self.eye.render(False, False, False) # render_depth, render_labeling, print_timing)
        self.rendered_rgb_eye = np.fromstring(rgb_eye, dtype=np.uint8).reshape( (self.EYE_H,self.EYE_W,3) )
   
    def set_eyeShow(self, to_show ):
        self.EYE_SHOW = to_show
        if(self.EYE_SHOW):
            self.eye_window = PygletInteractiveWindow(self, self.EYE_W, self.EYE_H)

        
    def set_eyeEnable(self, to_enable ):
        self.EYE_ENABLE = to_enable
        if not self.EYE_ENABLE: 
            self.EYE_SHOW = self.EYE_ENABLE
      
    def eye_adjust(self): 
        self.eye.move_and_look_at(0, 0, 1.5, 0, 0, 0)
