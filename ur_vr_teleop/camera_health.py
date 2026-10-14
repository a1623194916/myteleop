"""Live camera readiness tracking for teleoperation capture."""

import time


DEFAULT_CAMERA_TOPICS = (
    "/gripper/camera/color/image_raw",
    "/orbbec/color/image_raw",
)


class CameraHealth:
    def __init__(self, topics=DEFAULT_CAMERA_TOPICS, stale_timeout=1.0, clock=time.monotonic):
        if not topics or stale_timeout <= 0:
            raise ValueError("camera topics and a positive timeout are required")
        self.topics = tuple(topics)
        self.stale_timeout = float(stale_timeout)
        self.clock = clock
        self.last_frames = {}
        self.last_stamps = {}
        self.frame_counts = {}

    def observe(self, topic, message):
        if topic not in self.topics:
            raise ValueError("unexpected camera topic")
        if message.width <= 0 or message.height <= 0 or not message.data:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        previous_stamp = self.last_stamps.get(topic)
        if previous_stamp is not None and stamp <= previous_stamp:
            if stamp < previous_stamp:
                self.last_stamps[topic] = stamp
                self.frame_counts[topic] = 0
                self.last_frames.pop(topic, None)
            return
        now = self.clock()
        if topic not in self.last_frames or now - self.last_frames[topic] > self.stale_timeout:
            self.frame_counts[topic] = 0
        self.last_frames[topic] = now
        self.last_stamps[topic] = stamp
        self.frame_counts[topic] = self.frame_counts.get(topic, 0) + 1

    def problems(self):
        now = self.clock()
        problems = []
        for topic in self.topics:
            received = self.last_frames.get(topic)
            if received is None:
                problems.append(f"{topic}: 未收到有效图像")
            elif now - received > self.stale_timeout:
                problems.append(f"{topic}: 已 {now - received:.1f} 秒没有新图像")
            elif self.frame_counts.get(topic, 0) < 2:
                problems.append(f"{topic}: 等待连续新图像")
        return problems

    def subscribe(self, node):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        return [
            node.create_subscription(
                Image, topic,
                lambda message, camera_topic=topic: self.observe(camera_topic, message), qos,
            )
            for topic in self.topics
        ]
