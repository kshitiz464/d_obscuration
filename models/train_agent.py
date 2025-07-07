from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from models.drone_swarm_env import DroneSwarmEnv

def main():
    # Custom environment settings
    env_kwargs = {
        "num_drones": 7,
        "max_timesteps": 300,
        "dt": 0.1,
        "altitude": 10.0,
        "max_force": 20.0,
        "max_speed": 10.0,
        "collision_threshold": 1.0,
        "perception_radius": 50.0,
        "horizontal_spacing": 20.0,
        "vertical_spacing": 5.0,
        "wind_ax": 0.005,       # reduced wind effect
        "wind_ay": 0.005,
        "seed": 42
    }

    # Vectorized environment for parallel rollout collection
    env = make_vec_env(lambda: DroneSwarmEnv(**env_kwargs), n_envs=4)

    # Define PPO model
    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        tensorboard_log="./ppo_drone_tensorboard/",
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        gae_lambda=0.95,
        gamma=0.99,
        ent_coef=0.01,  # Encourage exploration
        vf_coef=0.5,
        max_grad_norm=0.5,
    )

    # Begin training
    model.learn(total_timesteps=500_000)
    model.save("models/ppo_drone_swarm")

    print("✅ Model training complete and saved.")

if __name__ == "__main__":
    main()


# from stable_baselines3 import PPO
# from stable_baselines3.common.env_util import make_vec_env
# import sys
# import os

# # Add parent directory to path to import custom environment
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
# from models.drone_swarm_env import DroneSwarmEnv


# def main():
#     # Core environment parameters only (no visual/logging extras)
#     env_kwargs = {
#         "num_drones": 5,
#         "max_timesteps": 500,
#         "dt": 0.1,
#         "horizontal_spacing": 20.0,
#         "vertical_spacing": 5.0,
#         "altitude": 10.0,
#         "collision_threshold": 1.0
#     }

#     # Create vectorized environment for PPO
#     env = make_vec_env(lambda: DroneSwarmEnv(**env_kwargs), n_envs=4)

#     # Define PPO model (no tensorboard/logging)
#     model = PPO (
#         policy="MlpPolicy",
#         env=env,
#         verbose=1,
#         tensorboard_log="./ppo_drone_tensorboard/",
#         ent_coef=0.02
#     )

#     # Train the model
#     model.learn(total_timesteps=100000)

#     # Save the trained model
#     model.save("models/ppo_drone_swarm")
#     print("Model training complete and saved.")


# if __name__ == "__main__":
#     main()
