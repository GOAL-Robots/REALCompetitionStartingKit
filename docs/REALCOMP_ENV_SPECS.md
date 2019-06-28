# REALCompetrition env specifications 

env:
  * __init__(render)
    * render
    
  * reset()
  
  * setCamera(self)
  
  * set_eye(self, name)
  
  * set_eye(self, name, eye_pos, target_pos)
    * name
    * eye_pos:     default [0.01, 0, 1.2]
    * target_pos:     default [0, 0, 0]
    
  * render()
    * mode='human'
    * close=False
    
  * get_part_pos(name)
    * name
    * returns pos
    
  * get_obj_pos(name)
    * name
    * returns pos

  * get_contacts()
    * return dict
    
  * get_retina()
    * return array


  * def get_observation()
    * returns dict
    
  * step(action)
      * returns tuple



