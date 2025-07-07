import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import gym
import numpy as np
from gym import spaces
from typing import List, Tuple

from sim_core.flocking_controller import FlockingController
from sim_core.formation_planner import FormationPlanner
from sim_core.sensor_utils import get_range

class DroneSwarmEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self,
                 num_drones: int = 5,
                 max_timesteps: int = 300,
                 dt: float = 0.1,
                 altitude: float = 10.0,
                 max_force: float = 20.0,
                 max_speed: float = 10.0,
                 collision_threshold: float = 1.0,
                 perception_radius: float = 50.0,
                 horizontal_spacing: float = 20.0,
                 vertical_spacing: float = 5.0,
                 wind_ax: float = 0.002,
                 wind_ay: float = 0.002,
                 seed: int = None):

        super(DroneSwarmEnv, self).__init__()

        self.num_drones = num_drones
        self.dt = dt
        self.max_timesteps = max_timesteps
        self.time_step = 0

        self.altitude = altitude
        self.max_force = max_force
        self.max_speed = max_speed
        self.collision_threshold = collision_threshold

        self.perception_radius = perception_radius
        self.horizontal_spacing = horizontal_spacing
        self.vertical_spacing = vertical_spacing

        self.wind_ax = wind_ax
        self.wind_ay = wind_ay

        self.seed_value = seed
        if seed is not None:
            self.seed(seed)

        self.flock = FlockingController(
            perception_radius=self.perception_radius,
            max_speed=self.max_speed,
            max_force=self.max_force,
            weight_sep=4.0,
            weight_align=1.0,
            weight_cohesion=1.0,
            weight_target=8.0,
            kp_target=0.8,
            nominal_spacing=5.0,
            hover_slowing_radius=25.0,
            vertical_hover_kp=20.0,
            vertical_hover_kd=30.0
        )

        self.observation_space = spaces.Box(
            low=-100.0,
            high=100.0,
            shape=(self.num_drones * 6,),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.num_drones * 3,),
            dtype=np.float32
        )

        self.reset()

    def seed(self, seed=None):
        np.random.seed(seed)

    def reset(self):
        self.positions = [
            np.array([np.random.uniform(-70, 70), np.random.uniform(-70, 70), 1.0])
            for _ in range(self.num_drones)
        ]
        self.velocities = [np.zeros(3) for _ in range(self.num_drones)]
        self.imu_deltas = [np.zeros(3) for _ in range(self.num_drones)]
        self.targets = FormationPlanner.staggered_pattern(
            self.num_drones, self.horizontal_spacing, self.vertical_spacing, self.altitude)

        self.time_step = 0
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate([np.concatenate((p, v)) for p, v in zip(self.positions, self.velocities)])

    def step(self, action: np.ndarray):
        forces = action.reshape((self.num_drones, 3)) * self.flock.max_force

        new_velocities = []
        new_positions = []
        collisions = 0
        total_dist = 0
        total_speed = 0

        for i in range(self.num_drones):
            vx, vy, vz = self.velocities[i]
            ax, ay, az = forces[i]

            # Wind effect
            ax += self.wind_ax
            ay += self.wind_ay

            drag = 0.005
            ax -= drag * vx
            ay -= drag * vy
            az -= drag * vz

            vx += ax * self.dt
            vy += ay * self.dt
            vz += az * self.dt

            speed = np.linalg.norm([vx, vy, vz])
            total_speed += speed
            if speed > self.flock.max_speed:
                vx, vy, vz = (np.array([vx, vy, vz]) * (self.flock.max_speed / speed)).tolist()

            px, py, pz = self.positions[i]
            px += vx * self.dt
            py += vy * self.dt
            pz += vz * self.dt

            new_positions.append(np.array([px, py, pz]))
            new_velocities.append(np.array([vx, vy, vz]))

        for i in range(self.num_drones):
            for j in range(i + 1, self.num_drones):
                if np.linalg.norm(new_positions[i] - new_positions[j]) < self.collision_threshold:
                    collisions += 1

        reward = 0
        reward -= collisions * 10

        for i in range(self.num_drones):
            dist = np.linalg.norm(new_positions[i] - np.array(self.targets[i]))
            total_dist += dist
            if dist < 1.0:
                reward += 2
            if np.linalg.norm(new_velocities[i]) < 0.5 and dist < 1.5:
                reward += 3

        reward -= 0.01 * total_dist
        reward -= 0.005 * total_speed
        reward -= 0.1

        self.positions = new_positions
        self.velocities = new_velocities
        self.time_step += 1

        done = self.time_step >= self.max_timesteps
        return self._get_obs(), reward, done, {}

    def render(self, mode="human"):
        print("Rendering not implemented. Use visualization separately if needed.")
