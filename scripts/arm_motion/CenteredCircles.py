#!/usr/bin/env python

import rospy
import math
import numpy as np
from open_abb_driver.srv import SetCartesianTrajectory, SetCartesianTrajectoryRequest
from percepto_msgs.msg import EpisodeBreak
from fieldtrack.srv import ResetFilter
from geometry_msgs.msg import Pose
from infitu.srv import StartEvaluation, StartTeardown
import broadcast
from abb_surrogate import wait_for_service, ArmMotionMonitor
from itertools import izip


class CenteredCircles:
    """Moves the arm back and forth at random speeds.
    """

    def __init__(self):
        set_pose_topic = rospy.get_param('~set_traj_service')
        wait_for_service(set_pose_topic)
        self.pose_service = rospy.ServiceProxy(set_pose_topic,
                                               SetCartesianTrajectory)

        info = rospy.get_param('~motion_monitor')
        self.motion_monitor = ArmMotionMonitor(**info)

        stream_name = rospy.get_param('~center_stream_name')
        self.center_tx = broadcast.Transmitter(stream_name=stream_name,
                                               feature_size=2,
                                               description='Next evaluation center, speed',
                                               mode='pull',
                                               namespace='~features')

        self.x_lims = rospy.get_param('~x_lims')
        self.y_lims = rospy.get_param('~y_lims')
        self.radius = rospy.get_param('~radius')
        self.num_loops = rospy.get_param('~num_loops')
        self.num_loop_points = rospy.get_param('~num_loop_waypoints')
        self.loop_interp_density = rospy.get_param('~loop_interp_density')
        self.z_value = rospy.get_param('~z_value')
        self.ori = rospy.get_param('~orientation')

        #self.step_time = rospy.get_param('~step_time')
        self.lower_speed = rospy.get_param('~min_speed')
        self.upper_speed = rospy.get_param('~max_speed')

        self.teardown_dt = rospy.get_param('~teardown_time')
        self.teardown_interp_density = rospy.get_param(
            '~teardown_interp_density')

        self.next_waypoint = [0, 0]
        self.next_dt = None
        self.waypoint_i = 0
        x = rospy.get_param('~waypoints_x')
        y = rospy.get_param('~waypoints_y')
        self.waypoints = [np.asarray(v) for v in zip(x, y)]
        for wp in self.waypoints:
            rospy.loginfo('Waypoint: %s', np.array_str(wp))
        self.dists = [np.linalg.norm(self.waypoints[i - 1] - self.waypoints[i])
                      for i in range(len(self.waypoints))]

        self.reset_service = None
        if rospy.has_param('~reset_filter_service'):
            reset_filter_topic = rospy.get_param('~reset_filter_service')
            wait_for_service(reset_filter_topic)
            self.reset_service = rospy.ServiceProxy(
                reset_filter_topic, ResetFilter)

        self.break_pub = rospy.Publisher('~breaks', EpisodeBreak,
                                         queue_size=10)

        self.evaluation_service = rospy.Service('~start_trajectory',
                                                StartEvaluation,
                                                self.evaluation_callback)
        self.teardown_service = rospy.Service('~start_teardown',
                                              StartTeardown,
                                              self.teardown_callback)

        self.__next_waypoint()

    def __next_waypoint(self):

        # Get next waypoint
        self.next_waypoint = self.waypoints[self.waypoint_i]
        self.waypoint_i += 1
        if self.waypoint_i >= len(self.waypoints):
            self.waypoint_i = 0

        # Sample next speed
        speed = np.random.uniform(low=self.lower_speed,
                                  high=self.upper_speed)
        self.next_dt = 2 * math.pi * self.radius / speed
        rospy.loginfo('Next center x: %f speed: %f',
                      self.next_waypoint[0],
                      speed)

        self.center_tx.publish(time=rospy.Time.now(),
                               feats=[self.next_waypoint[0], speed])

        # Move to next centerpoint
        self.__set_trajectory(xs=[self.next_waypoint[0] + self.radius],
                              ys=[self.next_waypoint[1]],
                              dts=[self.teardown_dt],
                              ns=[self.teardown_interp_density])
        self.motion_monitor.wait_for_stationary()

    def __exec_circle(self, c, r, dt, n_loops, n_points):
        theta = np.linspace(start=0, stop=n_loops * 2 * math.pi, num=n_points)
        x = r * np.cos(theta) + c[0]
        y = r * np.sin(theta) + c[1]
        dts = np.full(n_points, dt / n_points)
        ns = np.full(n_points, self.loop_interp_density)
        self.__set_trajectory(xs=x, ys=y, dts=dts, ns=ns)

    def __set_trajectory(self, xs, ys, dts, ns):
        req = SetCartesianTrajectoryRequest()
        req.interpolate = True

        xs = np.array(xs)
        ys = np.array(ys)
        if np.any(xs < self.x_lims[0]) or \
           np.any(xs > self.x_lims[1]) or \
           np.any(ys < self.y_lims[0]) or \
           np.any(ys > self.y_lims[1]):
            rospy.logerr('Violated bounds:\nx: %s\ny:%s', str(xs), str(ys))
            return None

        for x, y, dt, n in izip(xs, ys, dts, ns):
            pose = Pose()
            pose.position.z = self.z_value
            pose.orientation.w = self.ori[0]
            pose.orientation.y = self.ori[2]
            pose.orientation.z = self.ori[3]
            pose.orientation.x = self.ori[1]
            pose.position.x = x
            pose.position.y = y
            req.poses.append(pose)
            req.durations.append(dt)
            req.num_waypoints.append(n)
        try:
            self.pose_service(req)
        except rospy.ServiceException as e:
            rospy.logerr('Could not move to pose: ' + str(e))
            return None

    def evaluation_callback(self, srv):
        self.__evaluate()
        return []

    def teardown_callback(self, srv):
        self.__next_waypoint()
        return []

    def __evaluate(self):
        self.motion_monitor.wait_for_stationary()

        if self.reset_service is not None:
            self.reset_service()

        self.__exec_circle(c=self.next_waypoint,
                           r=self.radius,
                           dt=self.next_dt,
                           n_loops=self.num_loops,
                           n_points=self.num_loop_points)
        self.motion_monitor.wait_for_stationary()

        bmsg = EpisodeBreak()
        bmsg.break_time = rospy.Time.now()
        self.break_pub.publish(bmsg)


if __name__ == '__main__':
    rospy.init_node('random_speed_waypoints')
    try:
        rsw = CenteredCircles()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
