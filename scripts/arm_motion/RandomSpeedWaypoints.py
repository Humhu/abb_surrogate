#!/usr/bin/env python

import rospy
import math
import numpy as np
from open_abb_driver.srv import SetCartesianTrajectory, SetCartesianTrajectoryRequest
from percepto_msgs.msg import EpisodeBreak
from fieldtrack.srv import ResetFilter
from geometry_msgs.msg import Pose
from infitu.srv import StartEvaluation
import broadcast
from abb_surrogate import *
from itertools import izip


class RandomSpeedWaypoints:
    """Moves the arm back and forth at random speeds.
    """
    def __init__(self):
        set_pose_topic = rospy.get_param('~set_traj_service')
        wait_for_service(set_pose_topic)
        self.pose_service = rospy.ServiceProxy(set_pose_topic,
                                               SetCartesianTrajectory)

        info = rospy.get_param('~motion_monitor')
        self.motion_monitor = ArmMotionMonitor(**info)

        stream_name = rospy.get_param('~speed_stream_name')
        self.speed_tx = broadcast.Transmitter(stream_name=stream_name,
                                              feature_size=1,
                                              description='Next evaluation linear speed',
                                              mode='pull',
                                              namespace='~features')

        self.mode = rospy.get_param('~waypoint_mode')
        if self.mode == 'fixed_waypoints':
            x = rospy.get_param('~waypoints_x')
            y = rospy.get_param('~waypoints_y')
            self.waypoints = [np.asarray(v) for v in zip(x, y)]
            for wp in self.waypoints:
                rospy.loginfo('Waypoint: %s', np.array_str(wp))
            self.dists = [np.linalg.norm(self.waypoints[i - 1] - self.waypoints[i])
                          for i in range(len(self.waypoints))]
            self.lower_speed = rospy.get_param('~min_speed')
            self.upper_speed = rospy.get_param('~max_speed')
            self.next_speed = None
        elif self.mode == 'random_waypoints':
            x_lims = rospy.get_param('~x_lims')
            y_lims = rospy.get_param('~y_lims')
            self.lower_lims = (x_lims[0], y_lims[0])
            self.upper_lims = (x_lims[1], y_lims[1])
            self.step_time = rospy.get_param('~step_time')
            self.curr_waypoint = None
        else:
            raise ValueError('Unknown mode: %s' % self.mode)

        self.reset_service = None
        if rospy.has_param('~reset_filter_service'):
            reset_filter_topic = rospy.get_param('~reset_filter_service')
            wait_for_service(reset_filter_topic)
            self.reset_service = rospy.ServiceProxy(
                reset_filter_topic, ResetFilter)

        self.z_value = rospy.get_param('~z_value')
        self.ori = rospy.get_param('~orientation')

        self.num_waypoints = rospy.get_param('~num_pose_waypoints', 20)

        seed = rospy.get_param('~seed', None)
        rospy.loginfo('Seeding RNG with %s', str(seed))
        np.random.seed(seed)

        self.__initialize()
        self.__sample_next()

        self.break_pub = rospy.Publisher(
            '~breaks', EpisodeBreak, queue_size=10)

        trigger_mode = rospy.get_param('~trigger_mode')
        if trigger_mode == 'service':
            self.evaluation_service = rospy.Service('~start_trajectory',
                                                    StartEvaluation,
                                                    self.evaluation_callback)
        elif trigger_mode == 'timer':
            timer_rate = rospy.get_param('~spin_rate')
            self.timer = rospy.Timer(rospy.Duration(1.0 / timer_rate),
                                     self.timer_callback)
        elif trigger_mode == 'continuous':
            while not rospy.is_shutdown():
                self.__evaluate()
        else:
            raise ValueError('Unknown trigger mode: %s' % trigger_mode)

    def __initialize(self):
        if self.mode == 'fixed_waypoints':
            wp = self.waypoints[-1]
            # NOTE Hardcoded initialization slew time
            self.__set_trajectory([wp[0]], [wp[1]], [2.0])
        elif self.mode == 'random_waypoints':
            self.__set_trajectory([0], [0], [self.step_time])
            self.curr_waypoint = np.array((0, 0))

    def __sample_next(self):
        if self.mode == 'fixed_waypoints':
            self.next_speed = np.random.uniform(low=self.lower_speed,
                                                high=self.upper_speed,
                                                size=1)
            speed = self.next_speed
            rospy.loginfo('Next moving with speed %f' % speed)
        elif self.mode == 'random_waypoints':
            next_pose = np.random.uniform(low=self.lower_lims,
                                          high=self.upper_lims,
                                          size=2)
            dist = np.linalg.norm(next_pose - self.curr_waypoint)
            self.curr_waypoint = next_pose
            speed = dist / self.step_time
            rospy.loginfo('Next moving to %s with speed %f',
                          np.array_str(next_pose),
                          speed)
        self.speed_tx.publish(time=rospy.Time.now(), feats=[speed])

    def __set_trajectory(self, xs, ys, dts):
        req = SetCartesianTrajectoryRequest()
        req.interpolate = True

        for x, y, dt in izip(xs, ys, dts):
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
            req.num_waypoints.append(self.num_waypoints)
        try:
            self.pose_service(req)
        except rospy.ServiceException as e:
            rospy.logerr('Could not move to pose: ' + str(e))
            return None

    def evaluation_callback(self, srv):
        self.__evaluate()
        return []

    def timer_callback(self, event):
        self.__evaluate()

    def __evaluate(self):
        self.motion_monitor.wait_for_stationary()

        if self.reset_service is not None:
            self.reset_service()

        if self.mode == 'fixed_waypoints':
            xs, ys = izip(*self.waypoints)
            dts = [dist / self.next_speed for dist in self.dists]
            self.__set_trajectory(xs, ys, dts)
        elif self.mode == 'random_waypoints':
            self.__set_trajectory([self.curr_waypoint[0]],
                                  [self.curr_waypoint[1]],
                                  [self.step_time])
        self.motion_monitor.wait_for_stationary()

        bmsg = EpisodeBreak()
        bmsg.break_time = rospy.Time.now()
        self.break_pub.publish(bmsg)

        self.__sample_next()

if __name__ == '__main__':
    rospy.init_node('random_speed_waypoints')
    try:
        rsw = RandomSpeedWaypoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
