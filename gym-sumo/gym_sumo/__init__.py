from gym.envs.registration import register

# todo add other optional parameters

register(
    id='sumo-v0',
    entry_point='gym_sumo.envs:SUMOEnv',
)

# other entry
# register(
#     id='foo-extrahard-v0',
#     entry_point='gym_foo.envs:FooExtraHardEnv',
# )