#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec 19 15:51:48 2021

@author: anamaria
"""
from json.encoder import py_encode_basestring_ascii
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


vel = []
target_vel=[]
acc=[]
pos=[]
lat_vel=[]

action_names = ['LEFT LANE','IDLE','RIGHT LANE','FASTER','SLOWER']
df = pd.DataFrame(columns = ['vehicle', 'x','y','vx','vy'])
# Load and test saved model
model = DQN.load("highway_dqn/model")
done = False
t = 0
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
    
    v, tv, ac, p, lv = env.vehicleInfo()
    vel.append(v)
    target_vel.append(tv)
    acc.append(ac)
    pos.append([p[0],p[1]])
    lat_vel.append(lv)

    t+=1
        

step = np.arange(len(actions)) 
pos_ = np.array([np.array(xi) for xi in pos])

#df_ego = df.loc[0]
#df_ego['vel'] = ((df_ego['vx'])**2 + (df_ego['vy'])**2 )**(1/2) * 15
#vel_ = df_ego['vel'].to_numpy()

fig = plt.figure(figsize=(15, 11))

grid = plt.GridSpec(3, 2, wspace=0.4, hspace=0.3)

plt.subplot(grid[0, 0])
plt.title('Speed (m/s)')
plt.plot(range(t),vel,label = "Speed")
plt.step(range(t), target_vel, label = "Target")
plt.legend()
plt.grid()

# plt.subplot(3, 2, 2)
# plt.title('Steering angle (rad)')
# plt.plot(ran_t,heading, label = "Heading")
# plt.plot(ran_t,steering_angle, label = "Target")
# plt.legend()
# plt.grid()

plt.subplot(grid[1, 0])
plt.title('Acceleration (m/s²)')
plt.plot(range(t),acc)
plt.grid()

plt.subplot(grid[0, 1])
plt.title('Actions')
plt.yticks([0,1,2,3,4], action_names)
plt.step(range(t),actions)
plt.ylim(-1, 5) 
plt.grid()

x=pos_[:,0]
y=pos_[:,1]

plt.subplot(grid[1:, 1])
plt.title('Position (x,y)  (m)')
plt.plot(x,y)
plt.ylim(max(y)+30, min(y)-30)
plt.xlim(min(x)-30, max(x)+30)
plt.grid()

plt.show()