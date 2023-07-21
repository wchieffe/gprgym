import torch

from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.torch import tensor_clamp
from omni.isaac.gym.vec_env import VecEnvBase

from src.robots.franka import Franka
from src.robots.views.franka_view import FrankaView
from src.tasks.base.rl_task import RLTask
from src.utils.config_utils.sim_config import SimConfig


class GprBasketballTask(RLTask):
    def __init__(self, name: str, sim_config: SimConfig, env: VecEnvBase, offset=None) -> None:
        self._sim_config = sim_config
        self._cfg = sim_config.config
        self._task_cfg = sim_config.task_config

        # parse configurations, set task-specific members
        self._num_envs = self._task_cfg["env"]["numEnvs"]
        self._env_spacing = self._task_cfg["env"]["envSpacing"]
        self._max_episode_length = self._task_cfg["env"]["episodeLength"]
        self.action_scale = self._task_cfg["env"]["actionScale"]
        self.start_position_noise = self._task_cfg["env"]["startPositionNoise"]
        self.start_rotation_noise = self._task_cfg["env"]["startRotationNoise"]
        self.num_props = self._task_cfg["env"]["numProps"]
        self.joint_vel_scale = self._task_cfg["env"]["jointVelocityScale"]
        self.dist_reward_scale = self._task_cfg["env"]["distRewardScale"]
        self.rot_reward_scale = self._task_cfg["env"]["rotRewardScale"]
        self.around_handle_reward_scale = self._task_cfg["env"]["aroundHandleRewardScale"]
        self.open_reward_scale = self._task_cfg["env"]["openRewardScale"]
        self.finger_dist_reward_scale = self._task_cfg["env"]["fingerDistRewardScale"]
        self.action_penalty_scale = self._task_cfg["env"]["actionPenaltyScale"]
        self.finger_close_reward_scale = self._task_cfg["env"]["fingerCloseRewardScale"]
        self._num_observations = 4
        self._num_actions = 1

        # call parent class’s __init__
        RLTask.__init__(self, name, env)
        return

    def set_up_scene(self, scene: Scene) -> None:
        """implement environment setup here"""
        self.get_franka()
        super().set_up_scene(scene)
        self._frankas = FrankaView(prim_paths_expr="/World/envs/.*/franka", name="franka_view")
        scene.add(self._frankas)
        scene.add(self._frankas._hands)
        scene.add(self._frankas._lfingers)
        scene.add(self._frankas._rfingers)
        return

    def get_franka(self):
        franka = Franka(prim_path=self.default_zero_env_path + "/franka", name="franka")
        self._sim_config.apply_articulation_settings("franka", get_prim_at_path(franka.prim_path), self._sim_config.parse_actor_config("franka"))
        return

    def post_reset(self):
        """implement any logic required for simulation on-start here"""
        self.num_franka_joints = self._frankas.num_joint
        self.franka_joint_pos = torch.zeros((self.num_envs, self.num_franka_joints), device=self._device)
        joint_limits = self._frankas.get_joint_limits()
        self.franka_joint_lower_limits = joint_limits[0, :, 0].to(device=self._device)
        self.franka_joint_upper_limits = joint_limits[0, :, 1].to(device=self._device)
        self.franka_joint_speed_scales = torch.ones_like(self.franka_joint_lower_limits)
        self.franka_joint_speed_scales[self._frankas.gripper_indices] = 0.1
        self.franka_joint_targets = torch.zeros(
            (self._num_envs, self.num_franka_joints), dtype=torch.float, device=self._device
        )

        # randomize all envs
        indices = torch.arange(self._num_envs, dtype=torch.int64, device=self._device)
        self.reset_idx(indices)
        return

    def pre_physics_step(self, actions: torch.Tensor) -> None:
        """implement logic to be performed before physics steps"""
        if not self._env._world.is_playing():
            return

        reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_env_ids) > 0:
            self.reset_idx(reset_env_ids)

        # Set joint targets based on model's actions
        self.actions = actions.clone().to(self._device)
        targets = self.franka_joint_targets + self.franka_joint_speed_scales * self.dt * self.actions * self.action_scale
        self.franka_joint_targets[:] = tensor_clamp(targets, self.franka_joint_lower_limits, self.franka_joint_upper_limits)
        env_ids_int32 = torch.arange(self._frankas.count, dtype=torch.int32, device=self._device)
        self._frankas.set_joint_position_targets(self.franka_joint_targets, indices=env_ids_int32)
        return
    
    def reset_idx(self, env_ids):
        indices = env_ids.to(dtype=torch.int32)
        num_indices = len(indices)

        # reset franka
        pos = tensor_clamp(
            self.franka_default_joint_pos.unsqueeze(0)
            + 0.25 * (torch.rand((len(env_ids), self.num_franka_joints), device=self._device) - 0.5),
            self.franka_joint_lower_limits,
            self.franka_joint_upper_limits,
        )
        joint_pos = torch.zeros((num_indices, self._frankas.num_joint), device=self._device)
        joint_vel = torch.zeros((num_indices, self._frankas.num_joint), device=self._device)
        joint_pos[:, :] = pos
        self.franka_joint_targets[env_ids, :] = pos
        self.franka_joint_pos[env_ids, :] = pos
        self._frankas.set_joint_position_targets(self.franka_joint_targets[env_ids], indices=indices)
        self._frankas.set_joint_positions(joint_pos, indices=indices)
        self._frankas.set_joint_velocities(joint_vel, indices=indices)

        # bookkeeping
        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0
        return

    def get_observations(self) -> dict:
        """implement logic to retrieve observation states"""
        hand_pos, hand_rot = self._frankas._hands.get_world_poses(clone=False)
        drawer_pos, drawer_rot = self._cabinets._drawers.get_world_poses(clone=False)
        franka_joint_pos = self._frankas.get_joint_positions(clone=False)
        franka_joint_vel = self._frankas.get_joint_velocities(clone=False)
        self.cabinet_dof_pos = self._cabinets.get_joint_positions(clone=False)
        self.cabinet_dof_vel = self._cabinets.get_joint_velocities(clone=False)
        self.franka_dof_pos = franka_joint_pos
        self.franka_grasp_rot, self.franka_grasp_pos, self.drawer_grasp_rot, self.drawer_grasp_pos = self.compute_grasp_transforms(
            hand_rot,
            hand_pos,
            self.franka_local_grasp_rot,
            self.franka_local_grasp_pos,
            drawer_rot,
            drawer_pos,
            self.drawer_local_grasp_rot,
            self.drawer_local_grasp_pos,
        )

        self.franka_lfinger_pos, self.franka_lfinger_rot = self._frankas._lfingers.get_world_poses(clone=False)
        self.franka_rfinger_pos, self.franka_rfinger_rot = self._frankas._lfingers.get_world_poses(clone=False)

        joint_pos_scaled = (
            2.0
            * (franka_joint_pos - self.franka_joint_lower_limits)
            / (self.franka_joint_upper_limits - self.franka_joint_lower_limits)
            - 1.0
        )
        to_target = self.drawer_grasp_pos - self.franka_grasp_pos
        self.obs_buf = torch.cat(
            (
                joint_pos_scaled,
                franka_joint_vel * self.joint_vel_scale,
                to_target,
                self.cabinet_joint_pos[:, 3].unsqueeze(-1),
                self.cabinet_joint_vel[:, 3].unsqueeze(-1),
            ),
            dim=-1,
        )
        observations = {
            self._frankas.name: {
                "obs_buf": self.obs_buf
            }
        }
        return observations
    
    def compute_grasp_transforms(self, hand_rot, hand_pos, franka_local_grasp_rot, franka_local_grasp_pos, drawer_rot, drawer_pos, drawer_local_grasp_rot, drawer_local_grasp_pos):
        global_franka_rot, global_franka_pos = tf_combine(
            hand_rot, hand_pos, franka_local_grasp_rot, franka_local_grasp_pos
        )
        global_drawer_rot, global_drawer_pos = tf_combine(
            drawer_rot, drawer_pos, drawer_local_grasp_rot, drawer_local_grasp_pos
        )
        return global_franka_rot, global_franka_pos, global_drawer_rot, global_drawer_pos

    def calculate_metrics(self) -> None:
        """implement logic to compute rewards"""
        self.rew_buf = self.compute_rewards()

    def is_done(self) -> None:
        """implement logic to update dones/reset buffer"""
        self.reset_buf = self.compute_resets()
