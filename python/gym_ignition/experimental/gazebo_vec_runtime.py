# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym_ignition_models
from typing import Sequence
from gym_ignition import base
from gym_ignition.utils import logger
from gym_ignition.utils.typing import *
from gym_ignition.experimental import vec_runtime
from gym_ignition import scenario_bindings as bindings


class GazeboVecRuntime(vec_runtime.VecRuntime):

    # Differences with Runtime: autoreset, rendering, returned types -> vecenv.
    # We use the vecenv only for the interface, not for the sync / async feature (since
    # we don't have multiple gazebo calls).

    metadata = {'render.modes': ['human']}

    def __init__(self,
                 task_cls: type,
                 task_replicas: int,  # TODO
                 agent_rate: float,
                 physics_rate: float,
                 real_time_factor: float,
                 world: str = None,
                 **kwargs):

        # Gazebo attributes
        self._gazebo = None
        self._physics_rate = physics_rate
        self._real_time_factor = real_time_factor

        # World attributes
        self._world = None
        self._world_sdf = world

        if task_replicas <= 0:
            raise ValueError("Invalid number of task replicas")

        # Create the task objects
        tasks = [task_cls(agent_rate=agent_rate, **kwargs) for _ in range(task_replicas)]

        for task in tasks:
            if not isinstance(task, base.task.Task):
                raise RuntimeError("The task is not compatible with the runtime")

        # Wrap the tasks with the runtime
        super().__init__(tasks=tasks, agent_rate=agent_rate)

        # Trigger the initialization of the simulator and the worlds
        _ = self.gazebo

        # Create the spaces
        for task in self.tasks:
            task.action_space, task.observation_space = task.create_spaces()

        # Since all the tasks are objects of the same class, we expose to the agent
        # the spaces of one of them
        self.action_space = self.tasks[0].action_space
        self.observation_space = self.tasks[0].observation_space

        # Seed the environment
        self.seed()

        # Buffers
        self._states = None

    # =================
    # Runtime interface
    # =================

    def timestamp(self) -> float:

        now = self.tasks[0].world.time()

        for task in self.tasks:
            assert now == task.world.time()

        return now

    # ================================================
    # stable_baselines.common.vec_env.VecEnv interface
    # ================================================

    def step_async(self, actions: VecAction) -> None:

        # In our case the step is just one. In order to fully comply with APIs,
        # the actual Gazebo step should be executed in the step_wait method.
        # However, to make the environment randomizers work, we step the simulator here.

        # Set all the actions
        for action, task in zip(actions, self.tasks):

            if not self.action_space.contains(action):
                logger.warn("The action does not belong to the action space")

            # Set the action
            task.set_action(action)

        # Step the simulator
        ok_gazebo = self.gazebo.run()
        assert ok_gazebo, "Failed to run Gazebo"

        # Initialize the returned state
        self._states = (VecObservation([]), VecReward([]), VecDone([]), VecInfo([]))

        # Get all the states
        for task in self.tasks:
            observation, reward, done, info = self._get_task_state(task)

            # Store the state
            self._states[0].append(observation)
            self._states[1].append(reward)
            self._states[2].append(done)
            self._states[3].append(info)

    def step_wait(self) -> VecState:

        # Automatically reset terminated environments
        for idx, data in enumerate(zip(self.tasks, zip(*self._states))):

            # Unpack the data
            task, state = data

            # Unpack the state
            _, _, done, _ = state

            if done:
                # Automatically reset the task and override the observation
                # TODO: save last obs in dict -> check dummy_vec_env
                # TODO: check that this value is replaced in the list
                task.reset_task()
                observation = task.get_observation()

                # Replace the observation with the new one
                (self._states[0])[idx] = observation

                # Step the simulation to process reset changes
                ok_paused_run = self.gazebo.run(paused=True)

                if not ok_paused_run:
                    raise RuntimeError("Failed to execute a paused Gazebo run")

        return VecState(self._states)

    def reset(self) -> VecObservation:

        # This method will reset all the environment. The single envs get automatically
        # reset when they are done.

        # TODO
        # There is no way to reset an individual task without breaking the gym.Env APIs.
        # We follow the common logic used by other projects of automatically resetting
        # the task when they reach their terminal state.
        # The real reset operation happens in the step method. After reset, the task will
        # provide the agent the reward of the last step and new observation sampled from
        # its initial distribution.

        observations = VecObservation([])

        for task in self.tasks:

            task.reset_task()

            observation = task.get_observation()
            assert isinstance(observation, np.ndarray)

            observations.append(observation)

        return observations

    def render(self, mode: str = 'human', **kwargs) -> None:

        if mode == 'human':

            gui_ok = self.gazebo.gui()

            if not gui_ok:
                raise RuntimeError("Failed to render the environment")

            return

        raise Exception(f"Render mode '{mode}' not supported")

    def close(self) -> None:

        ok_close = self.gazebo.close()

        if not ok_close:
            raise RuntimeError("Failed to close Gazebo")

    def seed(self, seed: int = None) -> SeedList:

        # Returned seed list
        seed_list = SeedList([])

        # Create the seed if not passed
        seed = np.random.randint(2 ** 32 - 1) if seed is None else seed

        # Build the seed list
        seed_root = seed * len(self.tasks)
        seed_builder_list = [seed_root + task_idx for task_idx in range(len(self.tasks))]

        for task_seed, task in zip(seed_builder_list, self.tasks):

            # This method also seeds the spaces. To create them, the task could use
            # the world. Here we check that is has been created.
            if not task.has_world():
                raise RuntimeError("The world has never been created")

            # Seed the task (it will also seed the spaces)
            seed_list.append(task.seed_task(task_seed))

        return seed_list

    def get_attr(self, attr_name, indices=None):
        raise NotImplementedError

    def set_attr(self, attr_name, value, indices=None):
        raise NotImplementedError

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        raise NotImplementedError

    def get_images(self, *args, **kwargs) -> Sequence[np.ndarray]:
        raise NotImplementedError

    # ==============================
    # Private Methods and Properties
    # ==============================

    @staticmethod
    def _get_task_state(task: base.task.Task) -> State:

        # Get the observation
        observation = task.get_observation()
        assert isinstance(observation, np.ndarray)

        if not task.observation_space.contains(observation):
            logger.warn("The observation does not belong to the observation space")

        # Get the reward
        reward = task.get_reward()
        assert isinstance(reward, float), "Failed to get the reward"

        # Check termination
        done = task.is_done()

        return State((Observation(observation), Reward(reward), Done(done), Info({})))

    def _create_gazebo(self) -> bindings.GazeboSimulator:
        """
        Create an uninitialized instance of Gazebo
        """

        # Compute the number of physics iteration to execute at every environment step
        num_of_steps_per_run = self._physics_rate / self.agent_rate

        if num_of_steps_per_run != int(num_of_steps_per_run):
            logger.warn("Rounding the number of iterations to {} from the nominal {}"
                        .format(int(num_of_steps_per_run), num_of_steps_per_run))

        # Create the simulator
        gazebo = bindings.GazeboSimulator(1.0 / self._physics_rate,
                                          self._real_time_factor,
                                          int(num_of_steps_per_run))

        # Store the simulator
        self._gazebo = gazebo

        return gazebo

    @property
    def gazebo(self) -> bindings.GazeboSimulator:

        if self._gazebo is not None:
            assert self._gazebo.initialized()
            return self._gazebo

        # Store the simulator
        self._gazebo = self._create_gazebo()

        # Insert the worlds (it will initialize the simulator)
        self._create_worlds()
        assert self._gazebo.initialized()

        return self._gazebo

    def _create_worlds(self):

        if self._gazebo is None:
            raise RuntimeError("Gazebo has not yet been created")

        if self._gazebo.initialized():
            raise RuntimeError("Gazebo was already initialized, cannot insert world")

        worlds: Dict[base.task.Task, str] = {}

        if self._world_sdf is not None:
            world_filename = self._world_sdf
            world_name_base = bindings.getWorldNameFromSdf(self._world_sdf)
        else:
            world_filename = ""
            world_name_base = "default"

        for idx, task in enumerate(self.tasks):

            if idx == 0:
                worlds[task] = world_name_base
            else:
                worlds[task] = f"{world_name_base}{idx}"

            assert not task.has_world()

            ok_world = self._gazebo.insertWorldFromSDF(world_filename, worlds[task])

            if not ok_world:
                raise RuntimeError("Failed to load SDF world")

        # Initialize the simulator
        ok_initialize = self._gazebo.initialize()

        if not ok_initialize:
            raise RuntimeError("Failed to initialize Gazebo")

        if not self._gazebo.initialized():
            raise RuntimeError("Gazebo was not initialized")

        # Store the world in the task
        for task in self.tasks:
            task.world = self._gazebo.getWorld(worlds[task])
            print(task.world.name())

        # If no world file was passed, initialize the world with a ground plane and the
        # default physics plugin
        if self._world_sdf is None:

            for task in self.tasks:

                # Insert the ground plane
                ok_ground = task.world.insertModel(
                    gym_ignition_models.get_model_file("ground_plane"))

                if not ok_ground:
                    raise RuntimeError("Failed to insert the ground plane")

                # Insert the physics
                ok_physics = task.world.insertWorldPlugin(
                    "libPhysicsSystem.so", "scenario::plugins::gazebo::Physics")

                if not ok_physics:
                    raise RuntimeError("Failed to insert the physics plugin")
