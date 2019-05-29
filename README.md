# REALCompetitionStartingKit
Tested on Ubuntu (>= Ubuntu 16.04)

<TABLE " width="100%" BORDER="0">
<TR>
<TD><img src="docs/figs/demo0.gif" alt="demo0" width="100%"></TD>
<TD><img src="docs/figs/demo1.gif" alt="demo1" width="100%"></TD>
<TD><img src="docs/figs/demo2.gif" alt="demo1" width="100%"></TD>
</TR>
</TABLE>

### Install
cd path/to/realcomp 

#### Linux

To install the REAL Competition Starting Kit on linux

1) install gym and pybullet packages:

       pip install gym pybullet

2) download the REALCompetitionStartingKit repo:

       git clone https://github.com/GOAL-Robots/REALCompetitionStartingKit.git

3) install the REALCompetitionStartingKit package:

       cd REALCompetitionStartingKit
       pip install -e .

#### Windows - anaconda


To install the REAL Competition Starting Kit on windows in the anaconda enviroment

1) install microsoft Visual Studio c++14 - community at https://visualstudio.microsoft.com/visual-cpp-build-tools/

2) install the anaconda environment for windows at https://www.anaconda.com/distribution/#windows

3) create a python virtual environment

       conda create -n pyenv numpy pip

4) activate the virtual environment

       conda activate pyenv

3) install gym and pybullet packages:

       pip install gym pybullet

4) download the REALCompetitionStartingKit repo:

       git clone https://github.com/GOAL-Robots/REALCompetitionStartingKit.git

5) install the REALCompetitionStartingKit package:

       cd REALCompetitionStartingKit
       pip install -e .



### Basic usage

The environment is a standard gym environment and can be called alone as shownd here:

```python

env = gym.make('REALComp-v0')

observation = env.reset()  
for t in range(10):
    
    # Call your controller to chose action 
    action = controller.step(observation, reward, done)
    
    # do action
    observation, reward, done, _ = env.step(action)

```
The ```action```attribute  of ```env.step``` must be a  vector of 9 joint positions in radiants.


<TABLE " width="100%" BORDER="0">
<TR>
<TD>
       
| index |  joint name               |
| ----- | ------------------------- |
|  0    |  lbr_iiwa_joint_1         |
|  1    |  lbr_iiwa_joint_2         |
|  2    |  lbr_iiwa_joint_3         |
|  3    |  lbr_iiwa_joint_4         |
|  4    |  lbr_iiwa_joint_5         |
|  5    |  lbr_iiwa_joint_6         |
|  6    |  lbr_iiwa_joint_7         |
|  7    |  base_to_finger0_joint    |
|  8    |  finger0_to_finger1_joint |

</TD>
<TD><img src="docs/figs/kuka_full_joints.png" alt="demo1" width="100%"></TD>
<TD><img src="docs/figs/kuka_gripper_joints.png" alt="demo1" width="100%"></TD>
</TR>
</TABLE>


      


Using the env:

```python
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
            demo_run()

```

### Submission

### Evaluation

### Rules
