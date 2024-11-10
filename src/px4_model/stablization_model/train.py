# train.py

from agent import RLAgent
import argparse
import yaml

def main(config_path="config/agent_config.yaml", env_config_path="config/env_config.yaml", timesteps=500000, evaluate=True):
    # Initialize the RLAgent with the given configuration files
    agent = RLAgent(config_path=config_path, env_config_path=env_config_path)
    
    # Train the agent
    print("Starting training...")
    agent.train(timesteps=timesteps)
    print("Training completed.")

    # Optionally, evaluate the trained agent
    if evaluate:
        print("Evaluating the trained model...")
        agent.evaluate(num_episodes=5)

if __name__ == "__main__":
    # Argument parser for command-line options
    parser = argparse.ArgumentParser(description="Train and evaluate the RL agent")
    parser.add_argument("--config_path", type=str, default="config/agent_config.yaml", help="Path to the agent configuration file")
    parser.add_argument("--env_config_path", type=str, default="config/env_config.yaml", help="Path to the environment configuration file")
    parser.add_argument("--timesteps", type=int, default=500000, help="Total training timesteps")
    parser.add_argument("--evaluate", action="store_true", help="Evaluate the agent after training")
    
    args = parser.parse_args()

    # Run the main function with parsed arguments
    main(
        config_path=args.config_path,
        env_config_path=args.env_config_path,
        timesteps=args.timesteps,
        evaluate=args.evaluate
    )
