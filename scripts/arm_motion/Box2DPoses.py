#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
# from open_abb_driver.srv import SetSpeed, SetSpeedRequest
from open_abb_driver.srv import SetCartesianLinear, SetCartesianLinearRequest
from abb_surrogate.srv import SetSpeed
from paraset.srv import StartEvaluation


class Box2DPoses:

    def __init__(self):
        # set_speed_topic = rospy.get_param( '~speed_service' )
        set_pose_topic = rospy.get_param('~pose_service')
        # rospy.wait_for_service( set_speed_topic )
        rospy.wait_for_service(set_pose_topic)
        # self.speed_service = rospy.ServiceProxy( set_speed_topic, SetSpeed )
        self.pose_service = rospy.ServiceProxy(
            set_pose_topic, SetCartesianLinear)

        self.x_coords = rospy.get_param('~x_coords')
        self.y_coords = rospy.get_param('~y_coords')
        self.yaw_coords = rospy.get_param('~yaw_coords')

        self.z_value = rospy.get_param('~z_value')
        self.step_time = rospy.get_param('~step_time')
        self.num_waypoints = rospy.get_param('~num_waypoints')
        self.num_loops = rospy.get_param('~num_loops', 1)

        dwell_time = rospy.get_param('~dwell_time')
        self.loop_wait = rospy.Duration(dwell_time)

        self.waypoints = zip(self.x_coords, self.y_coords, self.yaw_coords)

        self.evaluation_service = rospy.Service('~start_trajectory', StartEvaluation,
                                                self.EvaluationCallback)
        self.speed_service = rospy.Service(
            '~set_speed', SetSpeed, self.SetSpeedCallback)

    def SetSpeedCallback(self, req):
        self.step_time = req.speed
        return []

    def EvaluationCallback(self, srv):
        for i in range(self.num_loops):
            for (x, y, yaw) in self.waypoints:
                rospy.loginfo('Traversing to x: %f, y: %f, yaw: %f', x, y, yaw)

                req = SetCartesianLinearRequest()
                req.pose.position.x = x
                req.pose.position.y = y
                req.pose.position.z = self.z_value
                req.pose.orientation.w = math.cos(yaw / 2)
                req.pose.orientation.x = 0
                req.pose.orientation.y = 0
                req.pose.orientation.z = math.sin(yaw / 2)
                req.duration = self.step_time
                req.num_waypoints = self.num_waypoints

                try:
                    self.pose_service(req)
                except rospy.ServiceException as e:
                    rospy.logerror('Could not query critique: ' + str(e))
                    return None

                rospy.sleep(self.loop_wait)
        return []


if __name__ == '__main__':
    rospy.init_node('random_2d_poses')
    try:
        r2dp = Box2DPoses()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
