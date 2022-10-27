import flxbot_commander
import sys

if __name__ == "__main__":
    commander = flxbot_commander.FlxbotCommander()
    state = commander.ros.state.state()
    segments = state.segments
    num_args = len(sys.argv)
    pin_vel = sys.argv[-1]
    rotate_vel = sys.argv[-2]
    args = sys.argv[1:(num_args - 2)]
    if (len(segments) * 2) != len(args):
        raise Exception
    pin_positions = []
    rotate_positions = []
    # segments = 2
    for idx, _ in enumerate(segments):
        rotate_positions.append(float(args[idx]))
        pin_positions.append(float(args[idx+1]))
    controllers = commander.controls.controllers
    controllers.joint_position.move(pin_positions, rotate_positions)
    controllers.joint_velocity.move(pin_positions, rotate_positions, float(pin_vel), float(rotate_vel))
    