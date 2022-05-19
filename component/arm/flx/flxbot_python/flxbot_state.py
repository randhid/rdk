# import flxbot_commander
import math

def _rad_to_deg(val):
    return (val * (180/math.pi))

if __name__ == "__main__":
    commander = flxbot_commander.FlxbotCommander()
    state = commander.ros.state.state()
    segments = state.segments
    result = []
    segments = [1, 1]
    for segment in segments:
        # result.append(_rad_to_deg(segment.drive.joints.rotate.position))
        # result.append(_rad_to_deg(segment.drive.joints.pin.position))   
        result.append(segment)
    print(" ".join([ str(el) for el in result ]))
