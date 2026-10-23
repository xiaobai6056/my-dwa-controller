#!/usr/bin/env python3
"""Set only the Gazebo client camera through its supported GUI transport topic."""
import subprocess
import time

MESSAGE = ('camera { name: "user_camera" pose { position { x: 2 y: 0 z: 8 } '
           'orientation { x: -0.5 y: 0.5 z: 0.5 w: 0.5 } } }')


def main():
    deadline = time.monotonic()+15
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(['gz', 'camera', '-l'], capture_output=True, text=True, timeout=3)
            if 'gzclient_camera' in result.stdout:
                subprocess.run(['gz', 'topic', '-p', '/gazebo/default/gui', '-m', MESSAGE],
                               check=True, timeout=5)
                print('Gazebo GUI camera set: world (2,0,8), looking down; no model state changed.')
                return 0
        except subprocess.TimeoutExpired:
            pass
        time.sleep(.2)
    print('Gazebo client camera not available in bounded startup window.')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())