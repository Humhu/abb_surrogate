#!/usr/bin/env python

import rospy
import random
import numpy as np

# from open_abb_driver.srv import SetSpeed, SetSpeedRequest
from open_abb_driver.srv import SetCartesianLinear, SetCartesianLinearRequest
from percepto_msgs.msg import EpisodeBreak
from fieldtrack.srv import ResetFilter

from ArmUtils import *

def wait_for_service(topic):
    rospy.loginfo('Waiting for service %s...', topic)
    rospy.wait_for_service(topic)
    rospy.loginfo('Connected to service%s!', topic)


class Random2DPoses:

    def __init__(self):

        set_pose_topic = rospy.get_param('~pose_service')
        wait_for_service(set_pose_topic)
        self.pose_service = rospy.ServiceProxy(
            set_pose_topic, SetCartesianLinear)

        self.reset_service = None
        if rospy.has_param('~reset_filter_service'):
            reset_filter_topic = rospy.get_param('~reset_filter_service')
            wait_for_service(reset_filter_topic)
            self.reset_service = rospy.ServiceProxy(reset_filter_topic, ResetFilter)

        if rospy.has_param('~seed'):
            seed = rospy.get_param('~seed')
            random.seed(seed)
            rospy.loginfo('Setting seed to %d', seed)
        
        info = rospy.get_param('~motion_monitor')
        self.motion_monitor = ArmMotionMonitor(**info)


        self.x_lims = rospy.get_param('~x_lims')
        if type(self.x_lims) is not list or len(self.x_lims) != 2:
            raise ValueError('Random2DPoses: x_lims must be [min,max]')

        self.y_lims = rospy.get_param('~y_lims')
        if type(self.y_lims) is not list or len(self.y_lims) != 2:
            raise ValueError('Random2DPoses: y_lims must be [min,max]')

        self.z_value = rospy.get_param('~z_value')
        self.ori = rospy.get_param('~orientation')
        self.step_time = rospy.get_param('~step_time')
        self.num_waypoints = rospy.get_param('~num_waypoints')
        self.steps_per_ep = rospy.get_param('~steps_per_ep',)

        self.pre_wait = rospy.Duration(rospy.get_param('~pre_dwell_time'))
        self.post_wait = rospy.Duration(rospy.get_param('~post_dwell_time'))        
        self.loop_wait = rospy.Duration(rospy.get_param('~inter_dwell_time'))        

        self.break_pub = rospy.Publisher(
            '~breaks', EpisodeBreak, queue_size=10)

    def Execute(self):
        while not rospy.is_shutdown():
            rospy.loginfo('Beginning episode...')
            bmsg = EpisodeBreak()
            bmsg.break_time = rospy.Time.now()
            self.break_pub.publish(bmsg)
            self.reset_service()

            rospy.sleep(self.pre_wait)
            for i in range(self.steps_per_ep):
                if rospy.is_shutdown():
                    return
                self.motion_monitor.wait_for_stationary()
                self.step()
                rospy.sleep(self.loop_wait)
            rospy.sleep(self.post_wait)
            rospy.loginfo('Episode complete.')

    def step(self):
        x = random.uniform(self.x_lims[0], self.x_lims[1])
        y = random.uniform(self.y_lims[0], self.y_lims[1])

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


if __name__ == '__main__':
    rospy.init_node('random_2d_poses')
    try:
        r2dp = Random2DPoses()
        r2dp.Execute()
    except rospy.ROSInterruptException:
        pass
