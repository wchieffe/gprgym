from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.gym.vec_env import VecEnvBase

from gprgym.robots.franka import Franka
from gprgym.robots.views.franka_view import FrankaView
from gprgym.tasks.base.rl_task import RLTask
from gprgym.utils.config_utils.sim_config import SimConfig


class GprBasketballTask(RLTask):
    def __init__(
        self,
        name: str,                # name of the Task
        sim_config: SimConfig,    # SimConfig instance for parsing cfg
        env: VecEnvBase,          # env instance of VecEnvBase or inherited class
        offset=None               # transform offset in World
    ) -> None:
        self._sim_config = sim_config
        self._cfg = sim_config.config
        self._task_cfg = sim_config.task_config

        # parse configurations, set task-specific members
        self._num_observations = 4
        self._num_actions = 1

        # call parent class’s __init__
        RLTask.__init__(self, name, env)

    def set_up_scene(self, scene: Scene) -> None:
        """implement environment setup here"""
        self.get_franka(my_robot) # add a robot actor to the stage
        super().set_up_scene(scene) # pass scene to parent class - this method in RLTask also uses GridCloner to clone the robot and adds a ground plane if desired
        self._frankas = FrankaView(prim_paths_expr="/World/envs/.*/franka", name="franka_view")
        scene.add(self._frankas) # add view to scene for initialization
        scene.add(self._frankas._hands)
        scene.add(self._frankas._lfingers)
        scene.add(self._frankas._rfingers)

    def get_franka(self):
        franka = Franka(prim_path=self.default_zero_env_path + "/franka", name="franka")
        self._sim_config.apply_articulation_settings("franka", get_prim_at_path(franka.prim_path), self._sim_config.parse_actor_config("franka"))

    def post_reset(self):
        """implement any logic required for simulation on-start here"""
        pass

    def pre_physics_step(self, actions: torch.Tensor) -> None:
        """implement logic to be performed before physics steps"""
        self.perform_reset()
        self.apply_action(actions)

    def get_observations(self) -> dict:
        """implement logic to retrieve observation states"""
        self.obs_buf = self.compute_observations()

    def calculate_metrics(self) -> None:
        """implement logic to compute rewards"""
        self.rew_buf = self.compute_rewards()

    def is_done(self) -> None:
        """implement logic to update dones/reset buffer"""
        self.reset_buf = self.compute_resets()
