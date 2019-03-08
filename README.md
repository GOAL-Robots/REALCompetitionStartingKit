# REALCompetitionStartingKit
Tested on Ubuntu (>= Ubuntu 16.04)

### Install
To install all needed stuff run:

    install/install.sh

### Usage
Using the env:
```python
        from OpenGL import GLU
        import roboschool
        import gym

        def demo_run():
        
            env = gym.make("RoboschoolKuka-v1")
            
            # uncomment these lines to enable the eye camera
            # env.unwrapped.set_eyeEnable(False)
            # env.unwrapped.set_eyeShow(False)
            
            # uncomment and/or change these lines to choose the objects to use
            # env.unwrapped.used_objects = ["BANANA", "HAMMER", "TOMATO_SOUP_CAN", 
            #                               "MUSTARD", "ORANGE"]

            obs = env.reset()  
    
            while True:
                # Call your controller to chose action 
                action = myRobotFunc(state, reward, info_)
                
                # do action
                state, reward, done, info_ = env.step(action)
                
                # render
                still_open = env.render("human")
                             
        if __name__=="__main__":
            demo_run()
```

<TABLE " BORDER="0">
<TR>
<TD><img src="docs/figs/demo0.gif" alt="demo0" width="200"></TD>
<TD><img src="docs/figs/demo1.gif" alt="demo1" width="200"></TD>
</TR><TR>
<TD><img src="docs/figs/demo2.gif" alt="demo2" width="200"></TD>
<TD><img src="docs/figs/demo3.gif" alt="demo3" width="200"></TD>
</TR>
</TABLE>
