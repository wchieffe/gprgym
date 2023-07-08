from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController, RMPFlowController
from omni.isaac.core.objects import DynamicCuboid
import numpy as np


def setup_scene():
    world = World()
    world.scene.add_default_ground_plane()
    fancy_cube =  world.scene.add(
        DynamicCuboid(
            prim_path="/World/random_cube",
            name="fancy_cube",
            position=np.array([0.3, 0.3, 0.3]),
            scale=np.array([0.0515, 0.0515, 0.0515]),
            color=np.array([0, 0, 1.0]),
        )
    )
    franka = world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
    world.reset()
    franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
    world.step(render=True) 
    return world, franka, fancy_cube


def pick_place(world, franka, fancy_cube):
    controller = PickPlaceController(
        name="pick_place_controller",
        gripper=franka.gripper,
        robot_articulation=franka,
    )
    while True:
        cube_position, _ = fancy_cube.get_world_pose()
        goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
        current_joint_positions = franka.get_joint_positions()
        actions = controller.forward(
            picking_position=cube_position,
            placing_position=goal_position,
            current_joint_positions=current_joint_positions,
        )
        franka.apply_action(actions)
        # Only for the pick and place controller, indicating if the state
        # machine reached the final state.
        if controller.is_done():
            world.pause()
            break
        world.step(render=True) 
    return


def move_to_position(world, franka):
    controller = RMPFlowController(
        name="rmp_flow_controller",
        robot_articulation=franka,
    )
    articulation_controller = franka.get_articulation_controller()
    waypoints = np.array([
        [0.5, 0.3, 0.2],
        [0.5, 0.4, 0.2],
        [0.5, 0.5, 0.2],
        [0.5, 0.6, 0.2],
        [0.5, 0.7, 0.2],
        [0.5, 0.8, 0.2],
        [0.5, 0.9, 0.2],
        [0.5, 1.0, 0.2],
        [0.5, 0.9, 0.2],
        [0.5, 0.8, 0.2],
        [0.5, 0.7, 0.2],
        [0.5, 0.6, 0.2],
        [0.5, 0.5, 0.2],
        [0.5, 0.4, 0.2],
        [0.5, 0.3, 0.2]
    ])
    for i in range(len(waypoints)):
        gripper_target_position = waypoints[i]
        while True:
            gripper_current_position = franka.gripper.get_world_pose()[0]
            diff = np.sum(np.abs(gripper_current_position - gripper_target_position))
            print("current position: ", gripper_current_position)
            print("target position: ", gripper_target_position)
            print("diff: ", diff)
            if diff < 0.1:
                break
            actions = controller.forward(
                target_end_effector_position = gripper_target_position
            )
            articulation_controller.apply_action(actions)
            world.step(render=True)
    return
