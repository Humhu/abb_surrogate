"""Tools for working with the ABB arm.
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from threading import Lock


def wait_for_service(srv):
    rospy.loginfo('Waiting for service %s to become available...', srv)
    rospy.wait_for_service(srv)
    rospy.loginfo('Service %s is now available!', srv)


class ArmMotionMonitor(object):
    """Monitors the arm pose topic to determine if the arm is moving.
    """

    def __init__(self, topic, tol, poll_rate):
        self._sub = rospy.Subscriber(topic, PoseStamped,
                                     self.pose_callback, queue_size=10)
        self._last_pose = None
        self._last_time = None
        self._tol = float(tol)
        self._moving = True
        self._poll_dur = rospy.Duration(1.0 / poll_rate)
        self._mutex = Lock()

    def pose_callback(self, msg):
        pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z,
                         msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w])

        self._mutex.acquire()
        if self._last_pose is not None:
            delta = pose - self._last_pose
            dt = (msg.header.stamp - self._last_time).to_sec()
            self._moving = np.linalg.norm(delta / dt) > self._tol
        self._last_pose = pose
        self._last_time = msg.header.stamp
        self._mutex.release()

    def wait_for_stationary(self):
        while self.is_moving():
            rospy.sleep(self._poll_dur)

    def is_moving(self):
        self._mutex.acquire()
        status = self._moving
        self._mutex.release()
        return status
