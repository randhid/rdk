#!/usr/bin/env python3

import argparse
import math
import flxbot_commander
import random
import rospy
import numpy as np
from scipy.spatial.transform import Rotation


def get_components(angle, plane_normal, offset_position):
    r = Rotation.from_rotvec(angle * np.array(plane_normal))
    return np.array(offset_position) + r.apply([1, 0, 0])


def get_angles(start_angle, end_angle, target_velocity, target_acceleration, time_step):
    current_velocity = 0
    angles = [start_angle]
    direction = 1.0 if end_angle > start_angle else -1.0
    while current_velocity < target_velocity:
        angles.append(
            angles[-1]
            + direction * current_velocity * time_step
            + direction * 0.5 * target_acceleration * time_step * time_step
        )
        current_velocity += target_acceleration * time_step
        print(f"{current_velocity}")
    current_velocity = target_velocity
    angular_difference = abs(angles[-1] - angles[0])
    while abs(end_angle - angles[-1]) > angular_difference:
        angles.append(angles[-1] + direction * target_velocity * time_step)

    end_offset = end_angle - start_angle
    while current_velocity * direction > 0:
        angles.append(
            angles[-1]
            + direction * current_velocity * time_step
            - direction * 0.5 * target_acceleration * time_step * time_step
        )
        current_velocity = max(current_velocity - target_acceleration * time_step, 0.0)
    angles.append(end_angle)
    return angles


def main():
    num_segments = len(flxbot_commander.ros.description.get_segment_names())
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-segments", type=int, default=num_segments)
    parser.add_argument("--plane", type=float, nargs=3, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--timestep", type=float, default=0.1)
    args = parser.parse_args()
    commander = flxbot_commander.FlxbotCommander(args.num_segments)

    num_moves = int(args.duration / args.timestep)
    link_length = 0.1868
    rospy.sleep(1.0)
    start_position_segment_1 = [link_length, 0, 0]

    start_angle = -math.pi / 4.0
    end_angle = math.pi / 4.0

    start_x, start_y, start_z = get_components(start_angle, args.plane, start_position_segment_1)
    print(f"Starting at: [{start_x}, {start_y}, {start_z}]")
    commander.controls.controllers.shape.move_to_poses(
        "base_link", [[0, 0, 0], start_position_segment_1, [start_x, start_y, start_z]]
    )
    rospy.loginfo("Waiting for move")
    input("Press enter when robot is ready")
    rospy.loginfo(f"Starting move with {num_moves} steps")

    desired_velocity = abs((end_angle - start_angle) / args.duration)
    desired_acceleration = desired_velocity / (args.duration * 0.2)

    for angle in get_angles(
        start_angle, end_angle, desired_velocity, desired_acceleration, args.timestep
    ):
        x, y, z = get_components(angle, args.plane, start_position_segment_1)
        print(f"Moving to: [{x:.3f}, {y:.3f}, {z:.3f}]")
        commander.controls.controllers.shape.move_to_poses(
            "base_link", [[0, 0, 0], start_position_segment_1, [x, y, z]]
        )
        rospy.sleep(args.timestep)
    input("Press enter when robot is ready to move back")
    rospy.loginfo("Waiting for move to end")
    for angle in get_angles(
        end_angle, start_angle, desired_velocity, desired_acceleration, args.timestep
    ):
        x, y, z = get_components(angle, args.plane, start_position_segment_1)
        print(f"Moving to: [{x:.3f}, {y:.3f}, {z:.3f}]")
        commander.controls.controllers.shape.move_to_poses(
            "base_link", [[0, 0, 0], start_position_segment_1, [x, y, z]]
        )
        rospy.sleep(args.timestep)
    rospy.sleep(1.0)


if __name__ == "__main__":
    main()
