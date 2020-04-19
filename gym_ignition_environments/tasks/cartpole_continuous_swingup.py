# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import numpy as np
from typing import Tuple
from gym_ignition.base import task
from gym_ignition import scenario_bindings as bindings
from gym_ignition.utils.typing import Action, Reward, Observation
from gym_ignition.utils.typing import ActionSpace, ObservationSpace


class CartPoleContinuousSwingup(task.Task, abc.ABC):

    def __init__(self,
                 agent_rate: float,
                 reward_cart_at_center: bool = True,
                 **kwargs):

        # Initialize the Task base class
        task.Task.__init__(self, agent_rate=agent_rate)

        # Name of the cartpole model
        self.model_name = None

        # Space for resetting the task
        self.reset_space = None

        # Private attributes
        self._reward_cart_at_center = reward_cart_at_center

        # Variables limits
        self._x_threshold = 2.4  # m
        self._dx_threshold = 20.0  # m /s
        self._q_threshold = np.deg2rad(5 * 360)  # rad
        self._dq_threshold = np.deg2rad(3 * 360)  # rad / s

    def create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:

        # Create the action space
        max_force = 200.0  # Nm
        action_space = gym.spaces.Box(low=np.array([-max_force]),
                                      high=np.array([max_force]),
                                      dtype=np.float32)

        # Configure reset limits
        high = np.array([
            self._x_threshold,   # x
            self._dx_threshold,  # dx
            self._q_threshold,   # q
            self._dq_threshold   # dq
        ])

        # Configure the reset space
        self.reset_space = gym.spaces.Box(low=-high,
                                          high=high,
                                          dtype=np.float32)

        # Configure the observation space
        obs_high = high.copy() * 1.2
        observation_space = gym.spaces.Box(low=-obs_high,
                                           high=obs_high,
                                           dtype=np.float32)

        return action_space, observation_space

    def set_action(self, action: Action) -> None:

        # Get the force value
        force = action.tolist()[0]

        # Set the force value
        model = self.world.getModel(self.model_name)
        ok_force = model.getJoint("linear").setGeneralizedForceTarget(force)

        if not ok_force:
            raise RuntimeError("Failed to set the force to the cart")

    def get_observation(self) -> Observation:

        # Get the model
        model = self.world.getModel(self.model_name)

        # Get the new joint positions and velocities
        q, x = model.jointPositions(["pivot", "linear"])
        dq, dx = model.jointVelocities(["pivot", "linear"])

        # Create the observation
        observation = Observation(np.array([x, dx, q, dq]))

        # Return the observation
        return observation

    def get_reward(self) -> Reward:

        # Get the model
        model = self.world.getModel(self.model_name)

        # Get the pendulum position
        q = model.getJoint("pivot").position()

        # Get the cart state
        x = model.getJoint("linear").position()
        dx = model.getJoint("linear").velocity()

        # Reward is [0, 1] for q=[0, pi]
        reward = (np.cos(q) + 1) / 2

        # Penalize cart velocities
        reward -= 0.1 * (dx ** 2)

        # Penalize positions close to the end of the rail
        reward -= 10.0 * (x >= 0.8 * self._x_threshold)

        return reward

    def is_done(self) -> bool:

        # Get the observation
        observation = self.get_observation()

        # The environment is done if the observation is outside its space
        done = not self.reset_space.contains(observation)

        return done

    def reset_task(self) -> None:

        if self.model_name not in self.world.modelNames():
            raise RuntimeError("Cartpole model not found in the world")

        # Get the model
        model = self.world.getModel(self.model_name)

        # Control the cart in force mode
        linear = model.getJoint("linear")
        ok_control_mode = linear.setControlMode(bindings.JointControlMode_Force)

        if not ok_control_mode:
            raise RuntimeError("Failed to change the control mode of the cartpole")

        # Create a new cartpole state
        q = np.pi - np.deg2rad(self.np_random.uniform(low=-60, high=60))
        x, dx, dq = self.np_random.uniform(low=-0.05, high=0.05, size=(3,))

        # Reset the cartpole state
        ok_reset_pos = model.resetJointPositions([x, q], ["linear", "pivot"])
        ok_reset_vel = model.resetJointVelocities([dx, dq], ["linear", "pivot"])

        if not ok_reset_pos and not ok_reset_vel:
            raise RuntimeError("Failed to reset the cartpole state")
