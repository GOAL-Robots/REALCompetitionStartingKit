from robot_bases import MJCFBasedRobot, URDFBasedRobot
import numpy as np
import pybullet_data
import os
from robot_bases import BodyPart


class Kuka(URDFBasedRobot):
    
    used_objects = [
            "table",
            "orange",
            "mustard",
            "hammer",
            "tomato"]

    object_poses = {
            "table":   [ 0.00,  0.00,  0.00, 0.00, 0.00, 0.00],
            "orange":  [-0.10,  0.00,  0.55, 0.00, 0.00, 0.00],
            "mustard": [ 0.00, -0.40,  0.55, 0.00, 0.00, 1.54],
            "hammer":  [ 0.00,  0.20,  0.55, 0.00, 0.00, 0.00],
            "tomato":  [-0.10,  0.40,  0.55, 0.00, 0.00, 0.00]}

    def __init__(self):
         
        self.action_dim = 9
        self.body_parts = 9 + 3 
        self.obs_dim = (9 + 3)*3
        
        URDFBasedRobot.__init__(self, 
                'kuka_gripper_description/urdf/kuka_gripper.urdf', 
                'kuka0', action_dim=self.action_dim, obs_dim=self.obs_dim)

        self.target = "orange"

        self.object_names = dict()
        self.object_bodies = dict()
        self.robot_parts = {}

 
    def reset(self, bullet_client):
        bullet_client.resetSimulation()
        super(Kuka, self).reset(bullet_client)
    
    def get_contacts(self):    
        contact_dict = {}
        for part_name, part in self.parts.items():         
            contacts= []
            for contact in part.contact_list():
                name = self.object_names[contact[2]] 
                if part_name in contact_dict.keys():
                    contact_dict[part_name].append(name)
                else:
                    contact_dict[part_name]= [name]  
        return contact_dict
    
    def robot_specific_reset(self, bullet_client):

        self.robot_body.reset_position([-0.8, 0, 0])


        self.object_bodies["kuka"] = self.robot_body
        self.object_names[0] = "kuka"

        for obj_name in self.used_objects:
            pos = self.object_poses[obj_name]
            obj = get_object(bullet_client,
                    "kuka_gripper_description/urdf/{}.urdf".format(obj_name),
                    *pos)
            self.object_bodies[obj_name] = obj
            self.object_names.update({obj.bodies[0]: obj_name})
        
        for _,joint in self.jdict.items():
            joint.reset_current_position(0, 0)

        for name, part in self.parts.items():
            self.robot_parts.update({part.bodyPartIndex: name})

    def apply_action(self, a):
        assert (np.isfinite(a).all())
        assert(len(a) == 9)

        a[:-2] = np.maximum(-np.pi*0.5, np.minimum(np.pi*0.5, a[:-2]))
        a[-2:] = np.maximum( 0, np.minimum(np.pi*0.5, a[-2:]))
        a[-1] = np.maximum( 0, np.minimum(2*a[-2], a[-1]))
 
        for i,j in enumerate(a[:-2]):
            self.jdict["lbr_iiwa_joint_%d"%(i+1)].set_position(j)
        
        self.jdict["base_to_finger00_joint"].set_position(a[-2])
        self.jdict["base_to_finger10_joint"].set_position(a[-2])
        self.jdict["finger00_to_finger01_joint"].set_position(-a[-1])
        self.jdict["finger10_to_finger11_joint"].set_position(-a[-1])

    def calc_state(self):
        state = np.hstack([part.get_position() for _,part in  self.parts.items()])
        state = np.hstack([state, self.object_bodies[self.target].get_position()])
        return state        
 
def get_object(bullet_client, object_file, x, y, z, roll=0, pitch=0, yaw=0):

    position = [x, y, z]
    orientation = bullet_client.getQuaternionFromEuler([roll, pitch, yaw])
    fixed = True
    body = bullet_client.loadURDF(
            fileName=os.path.join(pybullet_data.getDataPath(), object_file),
            basePosition=position,
            baseOrientation=orientation,
            useFixedBase=False,
            flags=bullet_client.URDF_USE_INERTIA_FROM_FILE)
    part_name, _ = bullet_client.getBodyInfo(body)
    part_name = part_name.decode("utf8")
    bodies = [body]
    return BodyPart(bullet_client, part_name, bodies, 0, -1)
