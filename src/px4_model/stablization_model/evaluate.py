# evaluate.py

from agent import RLAgent
import argparse

def main(model_path="./models/final_model.pth", config_path="config/agent_config.yaml", env_config_path="config/env_config.yaml", num_episodes=10):
    # Initialize the RLAgent with the given configuration files
    agent = RLAgent(config_path=config_path, env_config_path=env_config_path)

    # Load the pre-trained model
    agent.load_model(model_path=model_path)

    # Evaluate the agent
    print("Starting evaluation...")
    agent.evaluate(num_episodes=num_episodes)
    print("Evaluation completed.")

if __name__ == "__main__":
    # Argument parser for command-line options
    parser = argparse.ArgumentParser(description="Evaluate the RL agent")
    parser.add_argument("--model_path", type=str, default="./models/final_model.pth", help="Path to the trained model")
    parser.add_argument("--config_path", type=str, default="config/agent_config.yaml", help="Path to the agent configuration file")
    parser.add_argument("--env_config_path", type=str, default="config/env_config.yaml", help="Path to the environment configuration file")
    parser.add_argument("--num_episodes", type=int, default=10, help="Number of episodes to evaluate")
    
    args = parser.parse_args()

    # Run the main function with parsed arguments
    main(
        model_path=args.model_path,
        config_path=args.config_path,
        env_config_path=args.env_config_path,
        num_episodes=args.num_episodes
    )
