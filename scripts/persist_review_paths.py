#!/usr/bin/env python3
"""Keep RViz review paths visible after bag playback finishes or clock resets."""

import copy
import threading

import rospy
from nav_msgs.msg import Path


class ReviewPathCache:
    def __init__(self):
        self.publish_hz = rospy.get_param("~publish_hz", 2.0)
        self.restamp_to_now = rospy.get_param("~restamp_to_now", True)
        self.path_specs = [
            (
                rospy.get_param("~input_path_topic", "/ekf/input_path"),
                rospy.get_param("~review_input_path_topic", "/ekf/review/input_path"),
                "input_path",
            ),
            (
                rospy.get_param("~ekf_path_topic", "/ekf/ekf_path"),
                rospy.get_param("~review_ekf_path_topic", "/ekf/review/ekf_path"),
                "ekf_path",
            ),
            (
                rospy.get_param("~gnss_path_topic", "/ekf/gnss_path"),
                rospy.get_param("~review_gnss_path_topic", "/ekf/review/gnss_path"),
                "gnss_path",
            ),
        ]

        self.lock = threading.Lock()
        self.latest = {}
        self.publishers = {}
        self.subscribers = []

        for source_topic, review_topic, key in self.path_specs:
            self.publishers[key] = rospy.Publisher(review_topic, Path, queue_size=1, latch=True)
            self.subscribers.append(
                rospy.Subscriber(
                    source_topic,
                    Path,
                    self._make_callback(key),
                    queue_size=1,
                    tcp_nodelay=True,
                )
            )

        rospy.loginfo(
            "Review path cache active: %s",
            ", ".join("{}->{}".format(src, dst) for src, dst, _ in self.path_specs),
        )

    def _make_callback(self, key):
        def callback(msg):
            with self.lock:
                self.latest[key] = msg
            self.publish_one(key)

        return callback

    def _review_copy(self, msg):
        if not self.restamp_to_now:
            return msg

        out = copy.deepcopy(msg)
        stamp = rospy.Time.now()
        if not stamp.is_zero():
            out.header.stamp = stamp
            for pose in out.poses:
                pose.header.stamp = stamp
        return out

    def publish_one(self, key):
        with self.lock:
            msg = self.latest.get(key)
        if msg is None:
            return
        self.publishers[key].publish(self._review_copy(msg))

    def spin(self):
        interval = 1.0 / max(self.publish_hz, 0.1)
        while not rospy.is_shutdown():
            for _, _, key in self.path_specs:
                self.publish_one(key)
            rospy.rostime.wallsleep(interval)


def main():
    rospy.init_node("persist_review_paths")
    ReviewPathCache().spin()


if __name__ == "__main__":
    main()
