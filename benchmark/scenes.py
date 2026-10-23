#!/usr/bin/env python3
"""Scene definitions and consistent rectangle-based Gazebo/occupancy-map resources."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

HERE = Path(__file__).resolve().parent
BRINGUP = Path('/opt/ros/humble/share/nav2_bringup')
RESOLUTION = .05


def load_scene(name):
    data = yaml.safe_load((HERE / 'scenarios.yaml').read_text())['scenarios'][name]
    return dict(data, id=name)


def free_rectangles(scene):
    if 'free_rectangles' in scene:
        return scene['free_rectangles']
    rectangles = []
    half = scene['corridor_width'] / 2.
    for a, b in zip(scene['route'], scene['route'][1:]):
        if a[0] != b[0] and a[1] != b[1]:
            raise ValueError('only axis-aligned corridors are supported')
        rectangles.append([min(a[0], b[0])-half, min(a[1], b[1])-half,
                           max(a[0], b[0])+half, max(a[1], b[1])+half])
        # Asymmetric wall recesses add localization features without narrowing
        # the approved 1.6 m passage or changing its forced route.
        if scene.get('localization_recesses'):
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            distance = 1.
            while distance < length - .3:
                x = a[0] + (b[0]-a[0])*distance/length
                y = a[1] + (b[1]-a[1])*distance/length
                side = 1 if len(rectangles) % 2 else -1
                if a[1] == b[1]:
                    rectangles.append([x-.2, min(y+side*half, y+side*(half+.3)),
                                       x+.2, max(y+side*half, y+side*(half+.3))])
                else:
                    rectangles.append([min(x+side*half, x+side*(half+.3)), y-.2,
                                       max(x+side*half, x+side*(half+.3)), y+.2])
                distance += 1.5
    return rectangles


def merge_occupied(rows):
    active, rectangles = {}, []
    for y, row in enumerate(rows + [[False] * len(rows[0])]):
        runs, start = set(), None
        for x, cell in enumerate(row + [False]):
            if cell and start is None:
                start = x
            if not cell and start is not None:
                runs.add((start, x))
                start = None
        for run in list(active):
            if run not in runs:
                rectangles.append((run[0], active.pop(run), run[1], y))
        for run in runs:
            active.setdefault(run, y)
    return rectangles


def generate(scene, destination):
    free = free_rectangles(scene)
    ox = math.floor((min(r[0] for r in free)-1.) / RESOLUTION) * RESOLUTION
    oy = math.floor((min(r[1] for r in free)-1.) / RESOLUTION) * RESOLUTION
    nx = math.ceil((max(r[2] for r in free)+1.-ox) / RESOLUTION)
    ny = math.ceil((max(r[3] for r in free)+1.-oy) / RESOLUTION)
    rows = [[not any(a <= ox+(x+.5)*RESOLUTION <= c and
                     b <= oy+(y+.5)*RESOLUTION <= d for a, b, c, d in free)
             for x in range(nx)] for y in range(ny)]
    rectangles = [[ox+a*RESOLUTION, oy+b*RESOLUTION, ox+c*RESOLUTION, oy+d*RESOLUTION]
                  for a, b, c, d in merge_occupied(rows)]
    destination.mkdir(parents=True, exist_ok=True)
    pgm = destination / 'map.pgm'
    # Occupancy maps represent observed surfaces. Mark unobservable solid
    # interiors unknown, never free. Filling meter-thick boxes entirely occupied
    # makes AMCL's likelihood field reward laser endpoints deep inside a wall.
    classes = [[254 if not rows[y][x] else (0 if any(
        0 <= x+dx < nx and 0 <= y+dy < ny and not rows[y+dy][x+dx]
        for dx in (-1, 0, 1) for dy in (-1, 0, 1)) else 205)
        for x in range(nx)] for y in range(ny)]
    pgm.write_bytes(f'P5\n{nx} {ny}\n255\n'.encode() +
                    bytes(cell for row in reversed(classes) for cell in row))
    (destination / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': RESOLUTION, 'origin': [ox, oy, 0.],
        'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}))
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='default')
    for model in ('ground_plane', 'sun'):
        ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://' + model
    physics = ET.parse(BRINGUP / 'worlds/world_only.model').find('.//physics')
    world.append(physics)
    for i, (a, b, c, d) in enumerate(rectangles):
        model = ET.SubElement(world, 'model', name=f'wall_{i}')
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{(a+c)/2} {(b+d)/2} 0.5 0 0 0'
        link = ET.SubElement(model, 'link', name='wall')
        for tag in ('collision', 'visual'):
            geometry = ET.SubElement(ET.SubElement(link, tag, name=tag), 'geometry')
            ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = f'{c-a} {d-b} 1.0'
    ET.indent(sdf)
    ET.ElementTree(sdf).write(destination / 'world.sdf', encoding='unicode')
    result = {'origin': [ox, oy], 'resolution': RESOLUTION, 'occupied': rows,
              'obstacle_rectangles': rectangles, 'free_rectangles': free}
    (destination / 'geometry.json').write_text(json.dumps(
        {k: v for k, v in result.items() if k != 'occupied'}, indent=2) + '\n')
    return result


def resources(scene):
    if scene['world'] == 'stock':
        return BRINGUP / 'worlds/world_only.model', BRINGUP / 'maps/turtlebot3_world.yaml'
    directory = HERE / 'assets' / scene['id']
    if not (directory / 'world.sdf').exists():
        raise FileNotFoundError('run python3 benchmark/scenes.py generate first')
    return directory / 'world.sdf', directory / 'map.yaml'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['generate', 'launch-args', 'initial-pose'])
    parser.add_argument('scene', nargs='?')
    args = parser.parse_args()
    if args.command == 'generate':
        for name in ('S02', 'S03', 'S05'):
            data = generate(load_scene(name), HERE / 'assets' / name)
            print(name, 'Gazebo boxes:', len(data['obstacle_rectangles']))
        return
    scene = load_scene(args.scene)
    x, y, yaw = scene['start']
    if args.command == 'launch-args':
        world, map_file = resources(scene)
        print(f'world:={world}\nmap:={map_file}\nx_pose:={x}\ny_pose:={y}\nyaw:={yaw}')
    else:
        covariance = [0.] * 36
        covariance[0] = covariance[7] = .25
        covariance[35] = .0685
        print(json.dumps({'pose': {'header': {'frame_id': 'map'}, 'pose': {'pose': {
            'position': {'x': x, 'y': y}, 'orientation': {
                'z': math.sin(yaw/2), 'w': math.cos(yaw/2)}}, 'covariance': covariance}}}))


if __name__ == '__main__':
    main()
