# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
from gym_ignition import randomizers
from gym_ignition.utils import typing
from stable_baselines.common import vec_env
from gym_ignition.experimental import gazebo_vec_runtime


class GazeboVecTaskRandomizer(vec_env.VecEnvWrapper,
                              randomizers.base.task.TaskRandomizer,
                              abc.ABC):

    def __init__(self, venv: gazebo_vec_runtime.GazeboVecRuntime):

        if not isinstance(venv.unwrapped, gazebo_vec_runtime.GazeboVecRuntime):
            raise ValueError("The environment to wrap is not a GazeboVecRuntime")

        for idx, task in enumerate(venv.tasks):
            if task.world.id() == 0:
                raise RuntimeError(f"The world of task {idx} is not valid")

        super().__init__(venv=venv)

    def reset(self, **kwargs) -> typing.Observation:

        for task in self.venv.tasks:
            self.randomize_task(task, self.venv.gazebo, **kwargs)

        ok_paused_run = self.venv.gazebo.run(paused=True)

        if not ok_paused_run:
            raise RuntimeError("Failed to execute a paused Gazebo run")

        return self.venv.reset(**kwargs)

    def step_wait(self) -> typing.VecState:

        for task in self.venv.tasks:

            if task.is_done():
                self.randomize_task(task, self.venv.gazebo)

        states = self.venv.step_wait()

        return states
