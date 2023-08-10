from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, SubprocVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, BaseCallback
import csv
import numpy as np
import argparse
import json
import os

from KitchenEnv import KitchenEnv


def make_env(env_id):
    return KitchenEnv()

class CSVLoggerCallback(BaseCallback):
    def __init__(self, env_id, n_envs, csv_file_base, stats_file_base, verbose=0):
        super(CSVLoggerCallback, self).__init__(verbose)
        self.env_id = env_id
        self.csv_file = os.path.join(csv_file_base, f'env_{env_id}.csv')
        self.stats_file = os.path.join(stats_file_base, f'env_{env_id}.json')
        self.stats = {'total_reward': [], 'success': [], 'done': []}
        self.episode_rewards = np.zeros(n_envs)
        self.n_envs = n_envs
        with open(self.csv_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timesteps', 'total_reward', 'success', 'done'])

    def _on_step(self):
        for i, done in enumerate(self.locals['dones']):
            if i == self.env_id:  # Only write to the CSV file for the corresponding environment
                reward = self.locals['rewards'][i]
                self.episode_rewards[i] += reward
                if done:
                    success = self.locals['infos'][i].get('success', False)
                    with open(self.csv_file, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([self.num_timesteps, self.episode_rewards[i], success, done])
                    # Update stats
                    self.stats['total_reward'].append(self.episode_rewards[i])
                    self.stats['success'].append(success)
                    self.stats['done'].append(done)
                    self.episode_rewards[i] = 0
        return True

    def _on_training_end(self):
        # Save stats to JSON file
        with open(self.stats_file, 'w') as json_file:
            json.dump(self.stats, json_file)

    def reset(self):
        self.episode_rewards = np.zeros(self.n_envs)


def main():

    parser = argparse.ArgumentParser(description='Train a PPO model with parallel environments.')
    parser.add_argument('--n_envs', '-n', type=int, default=4, help='Number of parallel environments.')
    args = parser.parse_args()

    n_envs = args.n_envs  # Number of parallel environments
    
    # Create directories for logs
    os.makedirs('./logs/progress', exist_ok=True)
    os.makedirs('./logs/stats', exist_ok=True)
    os.makedirs('./logs/', exist_ok=True)
    os.makedirs('./ppo_kitchen_tensorboard/', exist_ok=True)

    # set the CSV logger callback
    csv_logger_callbacks = [CSVLoggerCallback(env_id=i, n_envs=n_envs, csv_file_base='./logs/progress', stats_file_base='./logs/stats') for i in range(n_envs)]

    # Create environments
    env = SubprocVecEnv([lambda i=i: make_env(i) for i in range(n_envs)])
    eval_env = SubprocVecEnv([lambda i=i: make_env(i) for i in range(n_envs)])


    # Normalize action and observation space
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)
    
    # Normalize
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=True, clip_obs=10.)

    # Create callbacks
    eval_callback = EvalCallback(eval_env, best_model_save_path='./logs/',
                                 log_path='./logs/', eval_freq=1000,
                                 deterministic=True, render=False)

    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./logs/',
                                            name_prefix='rl_model')

    tensorboard_log = "./ppo_kitchen_tensorboard/"

    # Initialize PPO model
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=tensorboard_log)

    # Train model
    model.learn(total_timesteps=25000, callback=[eval_callback, checkpoint_callback] + csv_logger_callbacks)

    # Save the model
    model.save("ppo_kitchen")

    # Don't forget to save the VecNormalize statistics when you save the agent
    env.save("ppo_kitchen_vec_normalize.pkl")

    # Evaluate the trained policy
    mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=10)

    print(f"mean_reward:{mean_reward:.2f} +/- {std_reward:.2f}")

    # Close the environment
    env.close()

if __name__ == '__main__':
    main()
