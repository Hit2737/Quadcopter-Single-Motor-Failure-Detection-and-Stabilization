# agent.py

import os
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from env.drone_env import DroneEnv
import yaml

class RLAgent:
    def __init__(self, config_path="config/agent_config.yaml", env_config_path="config/env_config.yaml"):
        # Load configurations
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        with open(env_config_path, 'r') as f:
            self.env_config = yaml.safe_load(f)
        
        # Initialize environment
        self.env = DroneEnv(self.env_config)

        # Set model parameters based on configuration
        self.model = PPO(
            "MlpPolicy",
            self.env,
            learning_rate=self.config["learning_rate"],
            n_steps=self.config["n_steps"],
            batch_size=self.config["batch_size"],
            n_epochs=self.config["n_epochs"],
            gamma=self.config["gamma"],
            verbose=1,
            tensorboard_log="./logs/tensorboard"
        )
        
    def train(self, timesteps=100000):
        # Callback to save checkpoints
        checkpoint_callback = CheckpointCallback(
            save_freq=10000,
            save_path="./models/checkpoints",
            name_prefix="ppo_drone"
        )
        
        # Train the model
        self.model.learn(total_timesteps=timesteps, callback=checkpoint_callback)
        
        # Save the final model
        model_path = "./models/final_model.pth"
        self.model.save(model_path)
        print(f"Model saved to {model_path}")

    def load_model(self, model_path="./models/final_model.pth"):
        # Load a pre-trained model
        if os.path.exists(model_path):
            self.model = PPO.load(model_path, env=self.env)
            print(f"Loaded model from {model_path}")
        else:
            print(f"Model path {model_path} does not exist.")

    def evaluate(self, num_episodes=10):
        # Evaluation loop
        for episode in range(num_episodes):
            obs = self.env.reset()
            done = False
            total_reward = 0
            
            while not done:
                action, _states = self.model.predict(obs, deterministic=True)
                obs, reward, done, info = self.env.step(action)
                total_reward += reward
            
            print(f"Episode {episode+1}: Total Reward = {total_reward}")

if __name__ == "__main__":
    agent = RLAgent()
    
    # Train the agent
    agent.train(timesteps=500000)
    
    # Evaluate the agent after training
    agent.evaluate(num_episodes=5)
