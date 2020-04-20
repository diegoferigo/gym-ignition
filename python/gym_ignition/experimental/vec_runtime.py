# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
from typing import List
from gym_ignition import base
from stable_baselines.common import vec_env


class VecRuntime(vec_env.VecEnv, gym.Env, abc.ABC):
    """
    """

    def __init__(self, tasks: List[base.task.Task], agent_rate: float):

        # TODO: check all tasks same
        # TODO: check at least one task

        #: List of the tasks handled by the runtime.
        self.tasks: List[base.task.Task] = tasks

        #: Rate of environment execution.
        self.agent_rate = agent_rate

        super().__init__(num_envs=len(self.tasks),
                         observation_space=tasks[0].observation_space,
                         action_space=tasks[0].action_space)

    @abc.abstractmethod
    def timestamp(self) -> float:
        """
        Return the timestamp associated to the execution of the environment.

        In real-time environments, the timestamp is the time read from the host system.
        In simulated environments, the timestamp is the simulated time, which might not
        match the real-time in the case of RTF different than 1.

        Returns:
            The current environment timestamp.
        """
