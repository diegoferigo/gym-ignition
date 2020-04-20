# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import time

# Register gym-ignition environments
import gym_ignition
import gym_ignition_environments
from gym_ignition import scenario_bindings
# from gym_ignition.utils import logger
from gym_ignition_environments.randomizers import cartpole

# Set gym verbosity
gym.logger.set_level(gym.logger.INFO)
assert gym.logger.set_level(gym.logger.DEBUG) or True
scenario_bindings.setVerbosity(1)
# scenario_bindings.setVerbosity(4)

# Create the environment
venv = gym.make("CartPoleDiscreteBalancing-VecGazebo-v0")

# Wrap the environment with the randomizer
# env = pendulum_swingup.PendulumSwingupEnvRandomizer(env=env)
venv = cartpole.CartpoleBalancingVecEnvRandomizer(venv=venv, seed=42)
number_of_parallel_envs = venv.unwrapped.spec._kwargs["task_replicas"]

# Enable the rendering
# TODO
# venv.render('human')
# time.sleep(3)

# Initialize the seed
venv.seed(42)

# Reset all the tasks
observations = venv.reset()
# print(observations)

# We will collect num_epochs * num_tasks samples.
# The tasks are automatically reset in the step() method when they terminate
# num_epochs = int(20000 / number_of_parallel_envs)
num_epochs = int(100 / number_of_parallel_envs)


class FPSCalculator():

    def __init__(self, average_time=1):
        self.counter = 0
        self.start = None
        self.average_time = average_time

    def add_samples(self, num_of_samples: int) -> None:

        if self.counter == 0:
            self.start = time.time()

        self.counter += num_of_samples

        now = time.time()
        elapsed = now - self.start

        if elapsed >= self.average_time:

            print(f"FPS={self.counter / elapsed}")

            # Reset the counter
            self.counter = 0


fps_calculator = FPSCalculator()

# while True:
    # start = time.time()
    # fps_calculator.add_samples(2)
    # time.sleep(0.0009)
    # stop = time.time()
    # elapsed = stop - start
    # if elapsed < 0.001:
    #     time.sleep(0.001 - elapsed)

for _ in range(num_epochs):
    # Execute a random action
    actions = [venv.action_space.sample() for _ in range(len(venv.tasks))]

    # start = time.time()
    observations, rewards, dones, _ = venv.step(actions)
    print(observations)
    # stop = time.time()

    # elapsed = stop - start
    # print(f"==> {number_of_parallel_envs / elapsed}")

    # print(len(observations))
    assert len(observations) == number_of_parallel_envs
    fps_calculator.add_samples(len(observations))
    # print(observations)

        # Render the environment
        # It is not required to call this in the loop
        # env.render('human')

        # Accumulate the reward
        # totalReward += reward

        # Print the observation
        # msg = ""
        # for value in observation:
        #     msg += "\t%.6f" % value
        # logger.debug(msg)

    # logger.info(f"Total reward for episode #{epoch}: {totalReward}")

venv.close()
time.sleep(5)
