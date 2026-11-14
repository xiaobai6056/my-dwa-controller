#!/usr/bin/env python3
"""Independent ROS 2 node for per-frame COCO person detection."""

from collections import deque
import json
from math import isfinite
from pathlib import Path
from threading import Condition, Thread
from time import monotonic, perf_counter

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
except ImportError:
    Detection2DArray = None

ROOT = Path(__file__).resolve().parent
TOPIC = "/visual_prototype"


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")
        weights = Path(self.declare_parameter("weights", str(ROOT / "weights/yolov8n.pt")).value)
        self.conf = self.declare_parameter("conf", 0.25).value
        self.imgsz = self.declare_parameter("imgsz", 640).value
        self.device = self.declare_parameter("device", "cuda:0").value
        image_topic = self.declare_parameter("image_topic", f"{TOPIC}/camera/image_raw").value
        if not isfinite(self.conf) or not 0.0 < self.conf <= 1.0:
            raise ValueError("conf must be in (0, 1]")
        if self.imgsz <= 0 or self.imgsz % 32:
            raise ValueError("imgsz must be a positive multiple of 32")
        if not weights.is_file():
            raise FileNotFoundError(weights)
        self.model = YOLO(str(weights), task="detect")
        if self.model.names.get(0) != "person":
            raise ValueError("Weights must define COCO class 0 as person")

        self.bridge = CvBridge()
        self.annotated = self.create_publisher(
            Image, f"{TOPIC}/annotated", qos_profile_sensor_data)
        self.metrics = self.create_publisher(String, f"{TOPIC}/metrics", 10)
        message_type = Detection2DArray or String
        self.detections = self.create_publisher(message_type, f"{TOPIC}/detections", 10)
        self.condition = Condition()
        self.latest = None
        self.stopping = False
        self.received = self.overwritten = self.failed = 0
        self.samples = deque(maxlen=300)
        self.report_start = monotonic()
        self.last_error = -float("inf")
        self.subscription = self.create_subscription(
            Image, image_topic, self.queue_frame, qos_profile_sensor_data)
        report_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.report_timer = self.create_timer(5.0, self.report, clock=report_clock)
        self.worker = Thread(target=self.infer_frames, name="yolo-inference")
        self.get_logger().info(
            f"{image_topic} -> {message_type.__name__}, conf={self.conf:g}, "
            f"imgsz={self.imgsz}, device={self.device}, weights={weights}")
        self.worker.start()

    def queue_frame(self, image):
        with self.condition:
            self.received += 1
            self.overwritten += self.latest is not None
            self.latest = image
            self.condition.notify()

    def infer_frames(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.latest is not None or self.stopping)
                if self.stopping:
                    return
                image, self.latest = self.latest, None
            try:
                elapsed, count = self.process_frame(image)
                with self.condition:
                    self.samples.append((elapsed, count))
            except Exception as error:
                with self.condition:
                    self.failed += 1
                now = monotonic()
                if not self.context.ok():
                    return
                if now - self.last_error >= 5:
                    self.get_logger().error(
                        f"Frame processing failed ({type(error).__name__}): {error}")
                    self.last_error = now

    def process_frame(self, image):
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        start = perf_counter()
        result = self.model.predict(
            frame, classes=[0], conf=self.conf, imgsz=self.imgsz,
            device=self.device, verbose=False)[0]
        boxes = [
            {"class_id": 0, "class_name": "person", "confidence": row[4], "xyxy": row[:4]}
            for row in result.boxes.data.cpu().tolist()
        ]
        elapsed = (perf_counter() - start) * 1000
        plotted = result.plot(conf=True, labels=True)
        annotated = self.bridge.cv2_to_imgmsg(
            cv2.cvtColor(plotted, cv2.COLOR_BGR2RGB), encoding="rgb8")
        annotated.header = image.header
        payload = String(data=json.dumps({
            "stamp": [image.header.stamp.sec, image.header.stamp.nanosec],
            "frame_id": image.header.frame_id,
            "predict_ms": elapsed,
            "forward_ms": result.speed["inference"],
            "boxes": boxes,
        }, separators=(",", ":")))
        self.annotated.publish(annotated)
        self.metrics.publish(payload)
        if Detection2DArray is None:
            self.detections.publish(payload)
        else:
            self.detections.publish(self.standard_message(image.header, boxes))
        return elapsed, len(boxes)

    @staticmethod
    def standard_message(header, boxes):
        message = Detection2DArray(header=header)
        for box in boxes:
            x1, y1, x2, y2 = box["xyxy"]
            detection = Detection2D(header=header)
            detection.bbox.center.position.x = (x1 + x2) / 2
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "0"
            hypothesis.hypothesis.score = box["confidence"]
            detection.results.append(hypothesis)
            message.detections.append(detection)
        return message

    def report(self):
        now = monotonic()
        with self.condition:
            samples = list(self.samples)
            self.samples.clear()
            received, overwritten, failed = self.received, self.overwritten, self.failed
            self.received = self.overwritten = self.failed = 0
        hz = len(samples) / (now - self.report_start)
        self.report_start = now
        if samples:
            average = sum(ms for ms, _ in samples) / len(samples)
            detected = sum(count > 0 for _, count in samples)
            self.get_logger().info(
                f"{hz:.1f} Hz, predict={average:.1f} ms, detected={detected}/{len(samples)}, "
                f"received={received}, overwritten={overwritten}, errors={failed}")
        else:
            self.get_logger().warning(f"No processed frames, received={received}, errors={failed}")

    def stop(self):
        with self.condition:
            self.stopping = True
            self.condition.notify()
        self.worker.join()


def main():
    rclpy.init()
    node = None
    try:
        node = PersonDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
