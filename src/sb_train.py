from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

from KitchenEnv import KitchenEnv

# Create environment
env = KitchenEnv()
env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run

# Normalize action and observation space
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

# Create callbacks
eval_callback = EvalCallback(env, best_model_save_path='./logs/',
                             log_path='./logs/', eval_freq=500,
                             deterministic=True, render=False)

checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./logs/',
                                         name_prefix='rl_model')

tensorboard_log = "./ppo_kitchen_tensorboard/"

# Initialize PPO model
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=tensorboard_log)

# Train model
model.learn(total_timesteps=2500, callback=[eval_callback, checkpoint_callback])

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
