from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, BaseCallback
import tensorboard
import csv
from KitchenEnv import KitchenEnv


class EpisodeEvalCallback(BaseCallback):
    def __init__(self, eval_env, eval_freq, verbose=0):
        super(EpisodeEvalCallback, self).__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.episode_count = 0

    def _on_step(self):
        if self.locals['dones'][0]:  # Assuming a single environment
            self.episode_count += 1
            if self.episode_count % self.eval_freq == 0:
                mean_reward, _ = evaluate_policy(self.model, self.eval_env, n_eval_episodes=10)
                print(f"Evaluation result: {mean_reward}")
        return True

class CSVLoggerCallback(BaseCallback):
    def __init__(self, csv_file, verbose=0):
        super(CSVLoggerCallback, self).__init__(verbose)
        self.csv_file = csv_file
        self.episode_rewards = 0
        # self.episode_steps = 0
        with open(self.csv_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timesteps', 'total_reward', 'success', 'done'])

    def _on_step(self):
        self.episode_rewards += self.locals['rewards'][0]
        # self.episode_steps += 1
        if self.locals['dones']:
            success = self.locals['infos'][0].get('success', False)
            with open(self.csv_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([self.num_timesteps, self.episode_rewards , success, self.locals['dones']])
            self.episode_rewards = 0
            # self.episode_steps = 0
        return True
    
    def reset(self):
        self.episode_rewards = 0
        # self.episode_steps = 0

# set the CSV logger callback
csv_logger_callback = CSVLoggerCallback('./logs/progress.csv')
# Create environment
env = KitchenEnv(callback=csv_logger_callback)
env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run

# Normalize action and observation space
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

# Create callbacks
eval_callback = EvalCallback(env, best_model_save_path='./logs/',
                             log_path='./logs/', eval_freq=1000,
                             deterministic=True, render=False)

checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./logs/',
                                         name_prefix='rl_model')

tensorboard_log = "./ppo_kitchen_tensorboard/"

# Initialize PPO model
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=tensorboard_log)

# Train model
model.learn(total_timesteps=25000, callback=[eval_callback, checkpoint_callback, csv_logger_callback])

# Save the model
model.save("ppo_kitchen")

# Don't forget to save the VecNormalize statistics when you save the agent
env.save("ppo_kitchen_vec_normalize.pkl")

# For loading the model and VecNormalize statistics:
# model = PPO.load("ppo_kitchen")
# env = VecNormalize.load("ppo_kitchen_vec_normalize.pkl", env)

# Evaluate the trained policy
mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)

print(f"mean_reward:{mean_reward:.2f} +/- {std_reward:.2f}")

# Close the environment
env.close()
