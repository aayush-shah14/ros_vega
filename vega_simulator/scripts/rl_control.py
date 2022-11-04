#!/usr/bin/env python3
import random,argparse,sys
parser = argparse.ArgumentParser()
import rospy
from math import ceil, pi
from std_msgs.msg import Float32MultiArray, Float32, Bool
from time import sleep
import numpy as np

import gym
from gym import spaces
#import stable_baselines3
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO, A2C # DQN coming soon
from stable_baselines3.common.env_util import make_vec_env

class vega_env(gym.Env):

  # Because of google colab, we cannot implement the GUI ('human' render mode)
      metadata = {'render.modes': ['console']}

      # Define constants for clearer code
      def __init__(self, n_nodes):
        super(vega_env, self).__init__()
        
        self.nodes=n_nodes
        self.l = 1.0
        #creating all publishers
        self.force_pub=rospy.Publisher("/actual_system/force",Float32MultiArray,queue_size=1)
        self.episode_reset_pub = rospy.Publisher("/actual_system/reset",Bool,queue_size=1)

        #creating all subscibers
        rospy.Subscriber("/actual_system/state",Float32MultiArray,self.state_callback,queue_size=1)


        #defining the messages and state varaibles
        self.forces_msg=Float32MultiArray()
        self.reset_msg = Bool()
        self.measured_state = np.array([0,0])
        self.target = np.array([-self.l/2,-self.l/2])

        # Define action and observation space
        self.observation_space = spaces.Box(low=-1*self.l, high=1*self.l, shape = (2,), dtype=np.float32)
        self.action_space = spaces.Box(low=0, high=50, shape = (2,), dtype=np.float32)
        # They must be gym.spaces objects

        #starting the controller by starting with zero input for first timestep so that step is obtained for next step
        self.forces_msg.data=np.zeros(shape=(2,))

      def state_callback(self,msg):

        #get current state
        rospy.loginfo('this is getting called')
        rospy.loginfo(msg.data)
        self.measured_state=np.array(msg.data).reshape((2,))

      def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array)
        """
        # Initialize the agent at position to default
        self.reset_msg.data = True
        self.episode_reset_pub.publish(self.reset_msg)
        
        rospy.Subscriber("/actual_system/state",Float32MultiArray,self.state_callback,queue_size=1) #check if callin subscriber is required here

        # observation should match observation space
        return np.array([self.measured_state[0],self.measured_state[1]]).astype(np.float32)

      def step(self, action):

        self.forces_msg.data=action
        self.force_pub.publish(self.forces_msg)
        #rospy.loginfo("controller node intialized and has Subscribed to compiled step and publishing to force")
        #rospy.loginfo("publishing first command to start the system")

        #rospy.Subscriber("/actual_system/state",Float32MultiArray,self.state_callback,queue_size=1) #check if callin subscriber is required here
        #rospy.spin()
        #subscribe to the state
        x = self.measured_state[0]
        y = self.measured_state[1]
        rospy.loginfo('this is getting called 2')

        distance = np.sqrt(((x - self.target[0])**2+ (y - self.target[1])**2))


        #check if we reached the goal
        done = bool(distance < 0.01)

        # Null reward everywhere except when reaching the goal (left of the grid)
        reward = - distance 
        info = {}
        return np.array([x, y]).astype(np.float32), reward, done, info

      # def render(self, mode='console'):
      #   if mode != 'console':
      #     raise NotImplementedError()
        # # agent is represented as a cross, rest as a dot
        # print("." * self.agent_pos, end="")
        # print("x", end="")
        # print("." * (self.grid_size - self.agent_pos))

      def close(self):
        pass


def callback(msg):
  rospy.loginfo(msg.data)
if __name__ == '__main__':

    rospy.init_node('Controller_node', anonymous = True)
    nodes= 405 
    env = vega_env(nodes)
    #obs = env.reset()
    #print(obs)
    # If the environment don't follow the interface, an error will be thrown
    check_env(env, warn=True)
    # wrap it
    #env = make_vec_env(lambda: env, n_envs=1)
    # Train the agent
    #model = A2C('MlpPolicy', env, verbose=1).learn(5000)
    obs = env.reset()
    n_steps = 200
    for step in range(n_steps):
      #action, _ = model.predict(obs, deterministic=True)
      action = np.array([2000,1900])
      print("Step {}".format(step + 1))
      print("Action: ", action)
      obs, reward, done, info = env.step(action)
      #rospy.Subscriber("/actual_system/state",Float32MultiArray,callback,queue_size=1)
      print('obs=', obs, 'reward=', reward, 'done=', done)
      #env.render(mode='console')
      if done:
    # Note that the VecEnv resets automatically
    # when a done signal is encountered
        print("Goal reached!", "reward=", reward)
        break
