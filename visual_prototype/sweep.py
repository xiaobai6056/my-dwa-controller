#!/usr/bin/env python3
"""Measure the live camera and detector for one candidate in Gazebo."""

import argparse
from collections import OrderedDict
import csv
import json
from math import cos, isclose, pi, sin
from pathlib import Path
from statistics import mean, median, quantiles
from time import monotonic

import cv2
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState, SetEntityState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from prepare_scene import CAMERA_X, CAMERA_Y, FRONT_YAW, OPTICAL

PREFIX = "/visual_prototype"
ANGLES = {"front": 0, "side": 90, "oblique": 45}
STATIC_FRAMES = 15


def stamp_of(message):
    stamp = message.header.stamp
    return stamp.sec, stamp.nanosec


def as_seconds(stamp):
    return stamp[0] + stamp[1] / 1e9


class Sweep(Node):
    def __init__(self, model, output):
        super().__init__("vision_sweep")
        self.model = model
        self.output = output
        self.bridge = CvBridge()
        self.images = OrderedDict()
        self.annotated = OrderedDict()
        self.results = OrderedDict()
        self.camera_info = None
        self.models = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.set_state = self.create_client(SetEntityState, f"{PREFIX}/set_entity_state")
        self.get_state = self.create_client(GetEntityState, f"{PREFIX}/get_entity_state")
        self.create_subscription(ModelStates, f"{PREFIX}/model_states", self.read_models, 10)
        self.create_subscription(
            CameraInfo, f"{PREFIX}/camera/camera_info", self.read_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, f"{PREFIX}/camera/image_raw",
            lambda image: self.remember(self.images, stamp_of(image), image),
            qos_profile_sensor_data)
        self.create_subscription(
            Image, f"{PREFIX}/annotated",
            lambda image: self.remember(self.annotated, stamp_of(image), image),
            qos_profile_sensor_data)
        self.create_subscription(String, f"{PREFIX}/metrics", self.read_result, 10)

    def read_models(self, message):
        self.models = message.name

    def read_info(self, message):
        self.camera_info = message

    def read_result(self, message):
        result = json.loads(message.data)
        self.remember(self.results, tuple(result["stamp"]), (monotonic(), result))

    @staticmethod
    def remember(cache, key, value):
        cache[key] = value
        if len(cache) > 45:
            cache.popitem(last=False)

    def spin_for(self, seconds):
        end = monotonic() + seconds
        while monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0, end - monotonic()))

    def request(self, client, request):
        result = client.call_async(request)
        rclpy.spin_until_future_complete(self, result, timeout_sec=3)
        if not result.done() or not result.result().success:
            raise RuntimeError(f"Gazebo failed to complete {client.srv_name}")
        return result.result()

    def set_pose(self, distance, angle, lateral=0.0):
        request = SetEntityState.Request()
        request.state.name = "pedestrian"
        request.state.reference_frame = "world"
        pose = request.state.pose
        pose.position.x = distance + CAMERA_X
        pose.position.y = CAMERA_Y + lateral
        yaw = FRONT_YAW[self.model] + angle * pi / 180
        pose.orientation.z = sin(yaw / 2)
        pose.orientation.w = cos(yaw / 2)
        self.request(self.set_state, request)
        return pose

    def verify_pose(self, expected):
        actual = self.request(
            self.get_state, GetEntityState.Request(name="pedestrian", reference_frame="world"))
        pose = actual.state.pose
        if any(not isclose(
            getattr(pose.position, axis), getattr(expected.position, axis), abs_tol=1e-4)
               for axis in "xyz"):
            raise RuntimeError(f"Unexpected Gazebo position: {pose.position}")
        dot = sum(getattr(pose.orientation, axis) * getattr(expected.orientation, axis)
                  for axis in "xyzw")
        if not isclose(abs(dot), 1, abs_tol=1e-4):
            raise RuntimeError(f"Unexpected Gazebo orientation: {pose.orientation}")
        return stamp_of(actual)

    def pair_frames(self, after, limit=None):
        rows = []
        for stamp in sorted(self.results.keys() & self.images.keys() & self.annotated.keys()):
            if stamp <= after:
                continue
            arrival, result = self.results[stamp]
            raw, annotated = self.images[stamp], self.annotated[stamp]
            if raw.header != annotated.header or raw.header.frame_id != result["frame_id"]:
                raise RuntimeError(f"Output header does not match the camera frame {stamp}")
            if (raw.header.frame_id != OPTICAL or (raw.width, raw.height) != (640, 480)
                    or raw.encoding != "rgb8" or annotated.encoding != "rgb8"
                    or (annotated.width, annotated.height) != (raw.width, raw.height)):
                raise RuntimeError(f"Unexpected RGB image contract at {stamp}")
            for box in result["boxes"]:
                x1, y1, x2, y2 = box["xyxy"]
                if not (box["class_id"] == 0 and 0 <= x1 < x2 <= raw.width
                        and 0 <= y1 < y2 <= raw.height and 0 < box["confidence"] <= 1):
                    raise RuntimeError(f"Invalid person box at {stamp}: {box}")
            rows.append((stamp, arrival, result, raw, annotated))
            if len(rows) == limit:
                break
        return rows

    def collect(self, after, count, timeout):
        end = monotonic() + timeout
        while monotonic() < end:
            rows = self.pair_frames(after, count)
            if len(rows) == count:
                return rows
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"Only {len(rows)} matched frames, expected {count}")

    def verify_camera(self):
        if not self.set_state.wait_for_service(timeout_sec=30):
            raise RuntimeError("Gazebo pose service is unavailable")
        if not self.get_state.wait_for_service(timeout_sec=5):
            raise RuntimeError("Gazebo state service is unavailable")
        self.collect((-1, 0), 4, 120)
        self.spin_for(0.35)
        info = self.camera_info
        if set(self.models or []) != {"ground_plane", "pedestrian", "turtlebot3_waffle"}:
            raise RuntimeError(
                f"Expected exactly one pedestrian and one Waffle, got {self.models}")
        if not (info and info.header.frame_id == OPTICAL
                and (info.width, info.height) == (640, 480)):
            raise RuntimeError("CameraInfo failed the image contract")
        transform = self.tf.lookup_transform("base_link", OPTICAL, Time()).transform
        xyz = [transform.translation.x, transform.translation.y, transform.translation.z]
        if any(not isclose(value, expected, abs_tol=1e-4)
               for value, expected in zip(xyz, (CAMERA_X, CAMERA_Y, 0.107))):
            raise RuntimeError(f"Unexpected camera TF translation: {xyz}")
        details = {
            "models": self.models, "frame_id": info.header.frame_id,
            "width": info.width, "height": info.height,
            "encoding": next(reversed(self.images.values())).encoding,
            "distortion_model": info.distortion_model, "K": list(info.k), "D": list(info.d),
            "base_link_to_optical_xyz": xyz,
            "base_link_to_optical_xyzw": [getattr(transform.rotation, axis) for axis in "xyzw"],
            "topics": dict(self.get_topic_names_and_types()),
        }
        (self.output / "interfaces.json").write_text(json.dumps(details, indent=2) + "\n")

    def save_images(self, case, row):
        _, _, _, raw, annotated = row
        for kind, image in (("raw", raw), ("annotated", annotated)):
            path = self.output / f"{case}-{kind}.png"
            if not cv2.imwrite(str(path), self.bridge.imgmsg_to_cv2(image, "bgr8")):
                raise OSError(f"Could not save {path}")

    def run(self):
        self.output.mkdir(parents=True, exist_ok=False)
        self.verify_camera()
        with (self.output / "frames.jsonl").open("w") as frames:
            summaries = []
            for distance in (1, 2, 3, 4):
                for angle, degrees in ANGLES.items():
                    case = f"{distance}m-{angle}"
                    expected = self.set_pose(distance, degrees)
                    self.spin_for(0.45)
                    watermark = self.verify_pose(expected)
                    rows = self.collect(watermark, STATIC_FRAMES, 20)
                    self.save_images(case, rows[len(rows) // 2])
                    summaries.append(summarize(frames, rows, case, distance, degrees))

            case = "2m-lateral"
            expected = self.set_pose(2, 0, -0.45)
            self.spin_for(0.45)
            watermark = self.verify_pose(expected)
            rows = []
            steps = 18
            with (self.output / "motion.csv").open("w", newline="") as motion:
                writer = csv.writer(motion)
                writer.writerow(("step", "sim_sec", "sim_nanosec", "x_m", "y_m", "z_m"))
                writer.writerow(
                    (0, *watermark, expected.position.x, expected.position.y, expected.position.z))
                for step in range(1, steps + 1):
                    expected = self.set_pose(2, 0, -0.45 + 0.90 * step / steps)
                    pose_stamp = self.verify_pose(expected)
                    writer.writerow((
                        step, *pose_stamp, expected.position.x, expected.position.y,
                        expected.position.z))
                    self.spin_for(0.14)
                    new_rows = self.pair_frames(watermark)
                    if new_rows:
                        rows.extend(new_rows)
                        watermark = new_rows[-1][0]
            if len(rows) < 10:
                raise RuntimeError(f"Only {len(rows)} frames during lateral motion")
            for name, index in (("start", 0), ("middle", len(rows) // 2), ("end", -1)):
                self.save_images(f"{case}-{name}", rows[index])
            summaries.append(summarize(frames, rows, case, 2, 0))

        with (self.output / "summary.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=summaries[0].keys())
            writer.writeheader()
            writer.writerows(summaries)


def summarize(stream, rows, case, distance, angle):
    boxes = []
    miss_run = max_miss_run = 0
    for stamp, arrival, result, _, _ in rows:
        stream.write(json.dumps(dict(result, case=case, wall_monotonic_s=arrival)) + "\n")
        if result["boxes"]:
            boxes.append(max(result["boxes"], key=lambda box: box["confidence"]))
            miss_run = 0
        else:
            miss_run += 1
            max_miss_run = max(miss_run, max_miss_run)
    stream.flush()
    xyxy = [round(median(box["xyxy"][i] for box in boxes), 1) for i in range(4)] if boxes else None
    span_wall = rows[-1][1] - rows[0][1]
    span_sim = as_seconds(rows[-1][0]) - as_seconds(rows[0][0])
    predict_ms = [row[2]["predict_ms"] for row in rows]
    print(f"{case}: {len(boxes)}/{len(rows)} detected, {mean(predict_ms):.2f} ms", flush=True)
    return {
        "case": case, "distance_m": distance, "yaw_offset_deg": angle,
        "frames": len(rows), "detections": len(boxes), "misses": len(rows) - len(boxes),
        "max_consecutive_misses": max_miss_run,
        "median_confidence": round(median(box["confidence"] for box in boxes), 4) if boxes else "",
        "median_xyxy_px": json.dumps(xyxy) if boxes else "",
        "median_width_px": (
            round(median(box["xyxy"][2] - box["xyxy"][0] for box in boxes), 1) if boxes else ""),
        "median_height_px": (
            round(median(box["xyxy"][3] - box["xyxy"][1] for box in boxes), 1) if boxes else ""),
        "mean_predict_ms": round(mean(predict_ms), 3),
        "p95_predict_ms": round(quantiles(predict_ms, n=100, method="inclusive")[94], 3),
        "mean_forward_ms": round(mean(row[2]["forward_ms"] for row in rows), 3),
        "matched_wall_hz": round((len(rows) - 1) / span_wall, 3),
        "matched_sim_hz": round((len(rows) - 1) / span_sim, 3),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", choices=FRONT_YAW)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Sweep(args.model, args.output)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
