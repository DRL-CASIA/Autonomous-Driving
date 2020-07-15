"""
All information on README.md
"""

import tensorflow as tf
from environment import Env
import numpy as np
import time
import model

steps = 1000
env = Env(vision=True)
ob = env.reset(relaunch=True)
print(ob)

###=================== Play the game with the trained model
# while True:
#     env = Env(vision=True)
#     ob = env.reset(relaunch=True)
#     loss = 0.0
#     for i in range(steps):
#         image = scipy.misc.imresize(ob, [66, 200]) / 255.0
#         degrees = model.y.eval(feed_dict={model.x: [image], model.keep_prob: 1.0})[0][0]
#         ob, reward, done, _ = env.step(act)
#         if done is True:
#             break
#         else:
#             ob_list.append(ob)
#
#     print("PLAY WITH THE TRAINED MODEL")
#     print(reward_sum)
#     env.end()
