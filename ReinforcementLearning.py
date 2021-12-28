#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec 19 15:51:48 2021

@author: anamaria
"""
import gym
import highway_env
from stable_baselines3 import DQN
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt 

env = gym.make("roundabout-v130")
# model = DQN('MlpPolicy', env,
#               policy_kwargs=dict(net_arch=[256, 256]),
#               learning_rate=5e-4,
#               buffer_size=15000,
#               learning_starts=200,
#               batch_size=32,
#               gamma=0.8,
#               train_freq=1,
#               gradient_steps=1,
#               target_update_interval=50,
#               verbose=1,
#               tensorboard_log="highway_dqn/")
# model.learn(int(2e4))
# model.save("highway_dqn/model")



actions = []
df = pd.DataFrame(columns = ['vehicle', 'x','y','vx','vy'])
# Load and test saved model
model = DQN.load("highway_dqn/model")
done = False
while done == False:
  done = False
  obs = env.reset()
  while not done:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    actions.append(action)
    env.render()
    dft = pd.DataFrame(obs,columns = ['vehicle', 'x','y','vx','vy'])
    df = df.append(dft)
    print(obs)
    #env.viewer.set_agent_action_sequence(action)

step = np.arange(len(actions)) 
# plt.step(step,actions)

df_ego = df.loc[0]
df_ego['vel'] = ((df_ego['vx'])**2 + (df_ego['vy'])**2 )**(1/2) * 15
vel = df_ego['vel'].to_numpy()
# plt.plot(step,vel)


fig, ax1 = plt.subplots()

ax2 = ax1.twinx()
ax1.step(step, actions, 'g-')
ax2.plot(step, vel, 'b-')

ax1.set_xlabel('Step')
ax1.set_ylabel('Action', color='g')
ax1.set_ylim(0,5)
ax1.set_ylim(0,20)
ax2.set_ylabel('Speed (m/s)', color='b')

plt.show()
    
    
    
    
    
    
    
    
    
    
    
    