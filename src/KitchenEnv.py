import os
import gym
from gym import spaces
import numpy as np
import pybullet as p
import pybullet_data  
import time
from colorama import Fore, Back, Style, init
init()
from kitchen import Kitchen


class KitchenEnv(gym.Env):
    def __init__(self, GUI=False, callback=None):
        super(KitchenEnv, self).__init__()
        self.GUI = GUI
        self.callback = callback
        if self.GUI: 
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        # self.client = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/models') 
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        p.setGravity(0, 0, -9.81)  
        self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
        self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
        self.table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)  
        # Define action and observation space
        # They must be gym.spaces objects
        # Box spaces for continuous actions and observations
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)  # Velocity of the drawer, can be positive (open) or negative (close)
        self.observation_space = spaces.Box(low=np.array([0]), high=np.array([1.8]), dtype=np.float32)  # Angle of the drawer, between 0 and 1.8 radians
        self.velocity_scaling_factor = 0.5  # Scaling factor for the velocity of the drawer
        self.done = False
        # Initialize the Kitchen
        self.kitchen = Kitchen()

    def step(self, action):
        # time.sleep(5)
        # Execute one time step within the environment
        self.done = False # Reset the done flag
        joint_id = self.kitchen.drawer_to_joint_id[1]  # Assuming 1 is the id of the drawer we want to control
        velocity = action[0] * self.velocity_scaling_factor  
        # print (Fore.CYAN + ("velocity = {}".format(velocity)) + Style.RESET_ALL)
        p.setJointMotorControl2(bodyIndex=self.kitchen.kitchen, jointIndex=joint_id, controlMode=p.VELOCITY_CONTROL, targetVelocity=velocity)
        p.stepSimulation() # used by pybullet to step the simulation
        joint_info = p.getJointState(self.kitchen.kitchen, joint_id)
        joint_position = joint_info[0]
        reward, success = self.compute_reward(joint_position, self.kitchen.drawer_to_joint_limits[1][1], self.kitchen.drawer_to_joint_limits[1][0], {})
        # if joint_position >= self.kitchen.drawer_to_joint_limits[1][1]:  # If the drawer is open
        #     reward = 1.0
        done = self.done # check the done after a step
        obs = self.get_observation()
        if done == True:
            self.reset()
            # reset the environment
        return obs, reward, done, {'success': success}

    def reset(self):
        # reset the loggers
        if self.callback is not None:
            self.callback.reset()
        # Reset the state of the environment to an initial state
        self.kitchen = Kitchen()
        return self.get_observation()

    def render(self, mode='human'):
        # Render the environment to the screen (optional)
        pass

    def get_observation(self):
        # Get the state of the drawer
        joint_info = p.getJointState(self.kitchen.kitchen, self.kitchen.drawer_to_joint_id[1])
        joint_position = joint_info[0]
        return np.array([joint_position])  # Current angle of the joint
    
    # def check_done(self):
    #     joint_info = p.getJointState(self.kitchen.kitchen, self.kitchen.drawer_to_joint_id[1])
    #     joint_position = joint_info[0]
    #     goal_position = self.kitchen.drawer_to_joint_limits[1][1]
    #     print ("joint_position = {} goal_position = {}".format(joint_position, goal_position))
    #     # time.sleep(0.1)
    #     return joint_position >= self.kitchen.drawer_to_joint_limits[1][1]
    
    def close(self):
        # Close the environment
        p.disconnect()

    def compute_reward(self, achieved_goal, desired_goal, undesired_goal, info = "sparse"):
        # Compute the reward given the current and desired goal
        # print (Fore.BLUE + ("achieved_goal = {} desired_goal = {}".format(achieved_goal, desired_goal)) + Style.RESET_ALL)
        reward = 0.0
        success = False
        if "dense" in info:
            reward = -np.linalg.norm(achieved_goal - desired_goal, axis=-1)
        else:
            if achieved_goal >= desired_goal:
                reward = 1.0
                success = True
                self.done = True # set done to be true: End episode and reach the goal
                # print (Fore.GREEN + "--------------------------------------------------------"+ Style.RESET_ALL)
                # print (Fore.GREEN + "----------------Reached the GOAL POSE------------------"+ Style.RESET_ALL)
                # print (Fore.GREEN + "--------------------------------------------------------"+ Style.RESET_ALL)
            elif achieved_goal <= undesired_goal: # if the drawer is closed
                reward = -0.1
                self.done = True
            else:
                reward = 0.0
        return reward, success

def main():
    # Create an instance of the environment
    env = KitchenEnv()

    # Reset the environment and get the initial observation
    obs = env.reset()

    # Take random actions for 100 steps
    for _ in range(100):
        # Sample a random action
        action = env.action_space.sample()

        # Take a step in the environment with the chosen action
        obs, reward, done, info = env.step(action)

        # If the episode is done, reset the environment
        if done:
            obs = env.reset()

        # Print the observation and reward
        print(f"Observation: {obs}, Reward: {reward}")

    # Close the environment
    env.close()

if __name__ == "__main__":
    main()
