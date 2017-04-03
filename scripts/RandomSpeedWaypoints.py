#!/usr/bin/env python

import rospy
import math
import numpy as np
from open_abb_driver.srv import SetCartesianLinear, SetCartesianLinearRequest
#from open_abb_driver.srv import SetSpeed, SetSpeedRequest
from open_abb_driver.srv import GetCartesian
from infitu.srv import StartEvaluation
import broadcast


def connect_service(srv):
    rospy.loginfo('Waiting for service %s...', srv)
    rospy.wait_for_service(srv)
    rospy.loginfo('Connected to service %s', srv)


class RandomSpeedWaypoints:

    def __init__(self):
        set_pose_topic = rospy.get_param('~set_pose_service')
        connect_service(set_pose_topic)
        self.pose_service = rospy.ServiceProxy(set_pose_topic,
                                               SetCartesianLinear)

        stream_name = rospy.get_param('~speed_stream_name')
        self.speed_tx = broadcast.Transmitter(stream_name=stream_name,
                                              feature_size=1,
                                              description='Next evaluation linear speed',
                                              mode='pull')

        x_lims = rospy.get_param('~x_lims')
        y_lims = rospy.get_param('~y_lims')
        self.lower_lims = (x_lims[0], y_lims[0])
        self.upper_lims = (x_lims[1], y_lims[1])

        self.z_value = rospy.get_param('~z_value')
        self.ori = rospy.get_param('~orientation')
        self.step_time = rospy.get_param('~step_time')

        self.num_waypoints = rospy.get_param('~num_pose_waypoints', 20)

        seed = rospy.get_param('~seed', None)
        rospy.loginfo('Seeing RNG with %s', str(seed))
        np.random.seed(seed)

        self.num_loops = rospy.get_param('~num_loops', 1)
        dwell_time = float(rospy.get_param('~dwell_time', 0.5))
        self.dwell_time = rospy.Duration(dwell_time)

        self.evaluation_service = rospy.Service('~start_trajectory',
                                                StartEvaluation,
                                                self.evaluation_callback)
        self.curr_pose = None
        self.__initialize()
        self.__sample_pose()

    def __initialize(self):
        self.__set_setpoint(0, 0)
        self.curr_pose = np.array((0, 0))

    def __sample_pose(self):
        next_pose = np.random.uniform(low=self.lower_lims, high=self.upper_lims,
                                      size=2)
        dist = np.linalg.norm(next_pose - self.curr_pose)
        self.curr_pose = next_pose
        speed = dist / self.step_time
        rospy.loginfo('Next moving to %s with speed %f',
                      np.array_str(next_pose),
                      speed)
        self.speed_tx.publish(time=rospy.Time.now(), feats=[speed])

    def __set_setpoint(self, x, y):
        rospy.loginfo('Traversing to x: %f, y: %f', x, y)

        req = SetCartesianLinearRequest()
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = self.z_value
        req.pose.orientation.w = self.ori[0]
        req.pose.orientation.x = self.ori[1]
        req.pose.orientation.y = self.ori[2]
        req.pose.orientation.z = self.ori[3]
        req.duration = self.step_time
        req.num_waypoints = self.num_waypoints

        try:
            self.pose_service(req)
        except rospy.ServiceException as e:
            rospy.logerr('Could not move to pose: ' + str(e))
            return None

    def evaluation_callback(self, srv):
        for i in range(self.num_loops):
            rospy.sleep(self.dwell_time)
            self.__set_setpoint(self.curr_pose[0], self.curr_pose[1])
        self.__sample_pose()
        return []


if __name__ == '__main__':
    rospy.init_node('random_speed_waypoints')
    try:
        r2dp = RandomSpeedWaypoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
