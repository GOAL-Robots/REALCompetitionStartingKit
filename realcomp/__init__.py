from gym.envs.registration import register

register(id='REALComp-v0', 
    entry_point='realcomp.envs:REALCompEnv', 
)

register(id='REALCompSingleObj-v0', 
    entry_point='realcomp.envs:REALCompEnvSingleObj', 
)

from realcomp.envs import realcomp_env
