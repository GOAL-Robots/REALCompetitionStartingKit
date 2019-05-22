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

pip install -e .

### Dependencies

pybullet

### Usage
Using the env:
```python
        from OpenGL import GLU
        import realcomp
        import gym

        def demo_run():
        
            env = gym.make('REALComp-v0')
            env.render('human')
            
            observation = env.reset()  
    
            while True:
                # Call your controller to chose action 
                action = myRobotFunc(state, reward, info_)
                
                # do action
                observation, reward, done, info = env.step(action)
                
                # get frame for storage 
                rgb_array = env.render('rgb_array')
                             
        if __name__=="__main__":
            demo_run()
```
