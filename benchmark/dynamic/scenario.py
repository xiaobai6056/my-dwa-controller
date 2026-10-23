#!/usr/bin/env python3
"""Generate deterministic physical scenes with the existing static map generator."""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from scenes import generate as generate_static  # noqa: E402


def load(name):
    return dict(
        json.loads(
            (HERE / 'scenarios.json').read_text())[name],
        id=name)


def position(scene, elapsed):
    frames = scene['keyframes']
    if elapsed <= frames[0][0]:
        return frames[0][1:]
    for a, b in zip(frames, frames[1:]):
        if elapsed <= b[0]:
            ratio = (elapsed - a[0]) / (b[0] - a[0])
            return [a[i] + ratio * (b[i] - a[i]) for i in (1, 2)]
    return frames[-1][1:]


def contact_sensor(link, collision, identifier):
    sensor = ET.SubElement(link, 'sensor', name='contacts', type='contact')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '100'
    ET.SubElement(
        ET.SubElement(
            sensor,
            'contact'),
        'collision').text = collision
    plugin = ET.SubElement(
        sensor,
        'plugin',
        name='contact_reporter_' +
        identifier,
        filename='libgazebo_ros_bumper.so')
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/benchmark'
    ET.SubElement(ros, 'remapping').text = 'bumper_states:=contacts'
    ET.SubElement(plugin, 'frame_name').text = 'world'


def generate(name, directory):
    scene = load(name)
    generate_static(scene, directory)
    tree = ET.parse(directory / 'world.sdf')
    world = tree.find('world')
    plugin = ET.SubElement(world, 'plugin', name='benchmark_state',
                           filename='libgazebo_ros_state.so')
    ET.SubElement(
        ET.SubElement(plugin, 'ros'),
        'namespace').text = '/benchmark'
    ET.SubElement(plugin, 'update_rate').text = '50'
    for wall in world.findall('model'):
        contact_sensor(wall.find('link'), 'collision', wall.get('name'))
    model = ET.SubElement(world, 'model', name='dynamic_obstacle')
    ET.SubElement(model, 'static').text = 'true'
    x, y = position(scene, 0)
    ET.SubElement(model, 'pose').text = f'{x} {y} {scene["size"][2]/2} 0 0 0'
    link = ET.SubElement(model, 'link', name='body')
    for tag in ('collision', 'visual'):
        element = ET.SubElement(link, tag, name=tag)
        box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = ' '.join(map(str, scene['size']))
    contact_sensor(link, 'collision', 'dynamic_obstacle')
    ET.indent(tree)
    tree.write(directory / 'world.sdf', encoding='unicode')
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'command',
        choices=[
            'generate',
            'launch-args',
            'initial-pose'])
    parser.add_argument('scene', choices=['D01', 'D02'], nargs='?')
    args = parser.parse_args()
    if args.command == 'generate':
        for name in ('D01', 'D02'):
            generate(name, HERE / 'assets' / name)
        return
    if args.command == 'launch-args':
        directory = HERE / 'assets' / args.scene
        print(f'world:={directory}/world.sdf\nmap:={directory}/map.yaml')
        print('x_pose:=0.0\ny_pose:=0.0\nyaw:=0.0')
    else:
        covariance = [0.] * 36
        covariance[0] = covariance[7] = .25
        covariance[35] = .0685
        print(json.dumps({'pose': {'header': {'frame_id': 'map'}, 'pose': {'pose': {
            'position': {'x': 0., 'y': 0.}, 'orientation': {'w': math.cos(0.)}},
            'covariance': covariance}}}))


if __name__ == '__main__':
    main()
