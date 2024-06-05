import math
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Type

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
import rclpy.qos
import rclpy.serialization
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, CameraInfo
from tf2_msgs.msg import TFMessage


def human_readable_size(bytes: float) -> str:
    suffixes = ["B", "KB", "MB", "GB"]

    if bytes < 1:
        return "0 B"

    log1000 = int(math.log10(bytes) // 3)
    return f"{bytes / (1000 ** log1000):.1f} {suffixes[log1000]}"


class ANSI:
    PREFIX = "\x1b["

    # Color sequences
    RED = PREFIX + "31m"
    GREEN = PREFIX + "32m"
    YELLOW = PREFIX + "33m"
    CYAN = PREFIX + "36m"
    WHITE = PREFIX + "37m"

    # SGR sequences
    RESET = PREFIX + "0m"
    BOLD = PREFIX + "1m"
    BLINK = PREFIX + "5m"
    NO_BOLD = PREFIX + "22m"
    NO_BLINK = PREFIX + "25m"

    # Control Sequences
    PREV_LINE = PREFIX + "1F"
    CLEAR_LINE = PREFIX + "2K"

    @classmethod
    def rgb(cls, s: str, r: int, g: int, b: int) -> str:
        return f"{cls.PREFIX}38;2;{r};{g};{b}m {s}{cls.RESET}"

    @classmethod
    def rgb_f(cls, s: str, r: float, g: float, b: float) -> str:
        return cls.rgb(s, int(r * 255), int(g * 255), int(b * 255))


@dataclass
class TopicStatistics:
    # wall time of this machine
    recv_times: deque[float] = field(default_factory=deque)
    sizes_bytes: deque[int] = field(default_factory=deque)
    total_size_bytes: int = field(default=0)

    def append(self, recv_time: float, size: int) -> None:
        self.recv_times.append(recv_time)
        self.sizes_bytes.append(size)
        self.total_size_bytes += size

    def prune(self, max_msg_age: float = 1.0) -> None:
        oldest_time = time.time() - max_msg_age

        while self.recv_times and self.recv_times[0] < oldest_time:
            self.recv_times.popleft()
            self.total_size_bytes -= self.sizes_bytes.popleft()


class TopicFrequencyNode(Node):
    def __init__(self) -> None:
        super().__init__("topic_frequency_monitor")

        # TODO(elvout): read topics and expected frequencies from config file
        topics: list[tuple[str, Type]] = [
            ("/camera/aligned_depth_to_color/camera_info", CameraInfo),
            ("/camera/aligned_depth_to_color/image_raw", Image),
            ("/camera/color/camera_info", CameraInfo),
            ("/camera/color/image_raw/compressed", CompressedImage),
            ("/camera/depth/color/points", PointCloud2),
            ("/tf", TFMessage),
        ]
        self.topic_stats: dict[str, TopicStatistics] = {}

        for topic_name, msg_type in topics:
            self.topic_stats[topic_name] = TopicStatistics()
            self.create_subscription(
                msg_type,
                topic_name,
                self.make_callback(topic_name),
                # In bandwidth-constrained network environments, the bandwidth
                # of the network may be less than a single message. Thus, we
                # used BEST_EFFORT instead of RELIABLE, otherwise no messages
                # will make it through the network.
                10,
                # rclpy.qos.QoSProfile(
                #     depth=10,
                #     history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                #     reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                # ),
                raw=True,
            )

        print(
            rclpy.qos.QoSProfile(
                **rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined(
                    "qos_profile_default"
                ).to_dict()
            )
        )
        print("----")
        print(rclpy.qos.qos_profile_system_default)
        print("----")
        print(rclpy.qos.qos_profile_sensor_data)
        print("----")

        self.create_timer(1, self.print_stats)

    def make_callback(self, topic_name: str) -> Callable:
        """
        rclpy won't let us use the same function reference as callbacks for
        multiple topics, so this creates a new function object but with the same
        logic.
        """

        def callback(raw_msg: bytes) -> None:
            self.topic_stats[topic_name].append(time.time(), len(raw_msg))

        return callback

    def print_stats(self) -> None:
        max_msg_age = 3.0
        total_bandwidth = 1e-2
        max_bandwidth = 1e-2

        for topic_name, stats in self.topic_stats.items():
            stats.prune(max_msg_age)

            total_bandwidth += stats.total_size_bytes
            max_bandwidth = max(max_bandwidth, stats.total_size_bytes)

        for topic_name, stats in self.topic_stats.items():
            bandwidth = stats.total_size_bytes

            bandwidth_s = f"{human_readable_size(bandwidth / max_msg_age):>8s}"
            green = 0.5 + 0.5 * math.sqrt(bandwidth / max_bandwidth)

            print(
                f"{topic_name:45s}: {len(stats.recv_times):4d} msgs | {len(stats.recv_times)/max_msg_age:6.1f} msg/s | {ANSI.rgb_f(bandwidth_s, 0.0, green, 1.0)}/s"
            )

        print(
            f"{ANSI.BOLD}{human_readable_size(total_bandwidth / max_msg_age):>83s}/s{ANSI.RESET}"
        )

        for _ in range(len(self.topic_stats) + 1):
            print(ANSI.PREV_LINE, end='')


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = TopicFrequencyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
