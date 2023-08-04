import os
import gym
from gym import spaces
import numpy as np
import pybullet as p
import pybullet_data  

from kitchen import Kitchen

class KitchenEnv(gym.Env):
    def __init__(self, GUI=False):
        super(KitchenEnv, self).__init__()
        self.GUI = GUI
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

        # Initialize the Kitchen
        self.kitchen = Kitchen()

    def step(self, action):
        # Execute one time step within the environment
        reward = 0.0
        joint_id = self.kitchen.drawer_to_joint_id[1]  # Assuming 1 is the id of the drawer we want to control
        velocity = action[0]
        p.setJointMotorControl2(bodyIndex=self.kitchen.kitchen, jointIndex=joint_id, controlMode=p.VELOCITY_CONTROL, targetVelocity=velocity)
        p.stepSimulation() # used by pybullet to step the simulation
        joint_info = p.getJointState(self.kitchen.kitchen, joint_id)
        joint_position = joint_info[0]
        if joint_position >= self.kitchen.drawer_to_joint_limits[1][1]:  # If the drawer is open
            reward = 1.0
        done = self.check_done()
        obs = self.get_observation()
        if done:
            print ("action = {} obs = {}  reward = {}  done = {}".format(action, obs, reward, done))
        return obs, reward, done, {}

    def reset(self):
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
    
    def check_done(self):
        joint_info = p.getJointState(self.kitchen.kitchen, self.kitchen.drawer_to_joint_id[1])
        joint_position = joint_info[0]
        return joint_position >= self.kitchen.drawer_to_joint_limits[1][1]
    
    def close(self):
        # Close the environment
        p.disconnect()
    
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
