"""Load V01 presets and reuse the prepared GEMS world for an isolated run."""

import argparse
import json
from math import isfinite
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
ROBOT = "turtlebot3_waffle"
PEDESTRIAN = "pedestrian"
MODES = ("A", "B", "C")


def load_mode(path, mode):
    config = json.loads(path.read_text())[mode]
    if set(config) != {"description", "robot", "pedestrian", "hold_s"}:
        raise ValueError("Each mode needs description, robot, pedestrian and hold_s")
    if not isinstance(config["description"], str):
        raise ValueError("description must be a string")
    for actor, velocity in (("robot", {"speed_mps"}), ("pedestrian", {"vx_mps", "vy_mps"})):
        data = config[actor]
        if set(data) != {"x_m", "y_m", "yaw_rad", "start_s", "duration_s"} | velocity:
            raise ValueError(f"Unexpected fields for {actor}")
        for key, value in data.items():
            validate_number(f"{actor}.{key}", value, nonnegative=key.endswith("_s"))
    validate_number("hold_s", config["hold_s"], nonnegative=True)
    robot, pedestrian = config["robot"], config["pedestrian"]
    robot_moves = bool(robot["speed_mps"])
    pedestrian_moves = bool(pedestrian["vx_mps"] or pedestrian["vy_mps"])
    if (robot_moves != bool(robot["duration_s"])
            or pedestrian_moves != bool(pedestrian["duration_s"])):
        raise ValueError("Duration must be positive exactly when the actor has nonzero velocity")
    if (robot_moves, pedestrian_moves) != (mode in ("B", "C"), mode in ("A", "C")):
        raise ValueError(f"Movement settings do not match mode {mode}")
    return config


def validate_number(name, value, nonnegative):
    if type(value) not in (float, int) or not isfinite(value) or (nonnegative and value < 0):
        qualifier = "nonnegative and finite" if nonnegative else "finite"
        raise ValueError(f"{name} must be a {qualifier} number")


def stop_time(actor):
    return actor["start_s"] + actor["duration_s"]


def events(config):
    timeline = [(0, "Scenario started")]
    for name in ("pedestrian", "robot"):
        actor = config[name]
        if actor["duration_s"]:
            timeline.extend([
                (actor["start_s"], f"{name.title()} starts moving"),
                (stop_time(actor), f"{name.title()} stops"),
            ])
    timeline.sort(key=lambda entry: entry[0])
    end = timeline[-1][0]
    return timeline + [(end, "Motion completed"), (end + config["hold_s"], "Scenario completed")]


def robot_speed(config, elapsed):
    robot = config["robot"]
    return robot["speed_mps"] if robot["start_s"] <= elapsed < stop_time(robot) else 0.0


def pedestrian_xy(config, elapsed):
    person = config["pedestrian"]
    dt = max(0.0, min(person["duration_s"], elapsed - person["start_s"]))
    return person["x_m"] + dt * person["vx_mps"], person["y_m"] + dt * person["vy_mps"]


def sim_label(seconds):
    minutes, tenths = divmod(round(seconds * 10), 600)
    return f"{minutes:02d}:{tenths / 10:04.1f}"


def prepare(mode, config_path, directory):
    config = load_mode(config_path, mode)
    tree = ET.parse(ROOT / ".cache/gems.world")
    world = tree.getroot().find("world")
    for entity, actor in ((ROBOT, config["robot"]), (PEDESTRIAN, config["pedestrian"])):
        pose = world.find(f"model[@name='{entity}']/pose")
        pose.text = f"{actor['x_m']} {actor['y_m']} 0 0 0 {actor['yaw_rad']}"
    pose_plugin = ET.SubElement(world, "plugin", name="v01_pose", filename=str(
        ROOT / ".cache/v01-gazebo-build/libv01_pose.so"))
    ET.SubElement(ET.SubElement(pose_plugin, "ros"), "namespace").text = "/visual_prototype"
    gui = ET.SubElement(world, "gui", fullscreen="false")
    camera = ET.SubElement(gui, "camera", name="v01_overview")
    ET.SubElement(camera, "pose").text = "1.65 -4.7 3.15 0 0.55 1.5707963268"
    ET.indent(tree)
    tree.write(directory / "v01.world", encoding="utf-8", xml_declaration=True)
    (directory / "config.json").write_text(
        json.dumps(
            {"mode": mode, "source": str(config_path.resolve()), **config}, indent=2) + "\n")
    print(f"V01 {mode}: {config['description']}")
    print(json.dumps(config, indent=2))
    print("Expected timeline (elapsed simulation time):")
    for seconds, event in events(config):
        print(f"  [SIM {sim_label(seconds)}] {event}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=MODES)
    parser.add_argument("config", type=Path)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    try:
        prepare(args.mode, args.config, args.run)
    except (KeyError, TypeError, ValueError, OSError, ET.ParseError) as error:
        parser.exit(2, f"V01 configuration error: {error}\n")


if __name__ == "__main__":
    main()
