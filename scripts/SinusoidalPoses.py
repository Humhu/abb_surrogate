#!/usr/bin/env python
import numpy as np
import rospy
import math
from itertools import izip
from geometry_msgs.msg import Pose
from open_abb_driver.srv import SetCartesianLinear, SetCartesianLinearRequest
from open_abb_driver.srv import SetCartesianTrajectory, SetCartesianTrajectoryRequest
from open_abb_driver.srv import ExecuteWaypoints
from open_abb_driver.srv import ClearWaypoints

def add_service(srv, ttype):
    topic = rospy.get_param(srv)
    print 'Waiting for %s...' % topic
    rospy.wait_for_service(topic)
    return rospy.ServiceProxy(topic, ttype)

class SinusoidalPoses:

    def __init__(self):

        self.traj_service = add_service('~traj_service', SetCartesianTrajectory)
        self.pose_service = add_service('~pose_service', SetCartesianLinear)
        self.exec_service = add_service('~exec_service', ExecuteWaypoints)
        self.clear_service = add_service('~clear_service', ClearWaypoints)

        self.x_amplitude = rospy.get_param('~x_amplitude')
        self.x_center = rospy.get_param('~x_center')
        self.y_amplitude = rospy.get_param('~y_amplitude')
        self.y_center = rospy.get_param('~y_center')
        self.z_value = rospy.get_param('~z_value')
        self.ori = rospy.get_param('~orientation')

        frequency = rospy.get_param('~traj_frequency')
        self.k = 2 * math.pi * frequency
        period = 1.0 / frequency
        self.num_control_points = rospy.get_param('~num_control_points', 30)
        if self.num_control_points < 2:
            raise ValueError('Must have at least 2 control points')

        control_dt = period / self.num_control_points
        t_values = np.linspace(start=0, stop=period, num=self.num_control_points)
        x_values, y_values = self.get_pos(t_values)
        print x_values
        self.traj_req = SetCartesianTrajectoryRequest()
        for x, y in izip(x_values, y_values):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self.z_value
            pose.orientation.w = self.ori[0]
            pose.orientation.x = self.ori[1]
            pose.orientation.y = self.ori[2]
            pose.orientation.z = self.ori[3]
            self.traj_req.poses.append(pose)
            self.traj_req.durations.append(control_dt)

        self.num_periods = rospy.get_param('~num_periods', 1)

    def get_pos(self, t):
        x = self.x_center + self.x_amplitude * np.cos(t * self.k)
        y = self.y_center + self.y_amplitude * np.cos(t * self.k)
        return x, y

    def slew(self, x, y, dur):
        req = SetCartesianLinearRequest()
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = self.z_value
        req.pose.orientation.w = self.ori[0]
        req.pose.orientation.x = self.ori[1]
        req.pose.orientation.y = self.ori[2]
        req.pose.orientation.z = self.ori[3]
        req.duration = dur
        req.num_waypoints = self.num_control_points

        try:
            rospy.loginfo('Slewing to %f %f', x, y)
            self.pose_service(req)
        except rospy.ServiceException as e:
            rospy.logerr('Could not slew arm: %s', str(e))

    def execute(self):
        init_x, init_y = self.get_pos(0)
        rospy.loginfo('Slewing arm to initial position...')
        self.slew(init_x, init_y, 3.0)

        try:
            self.clear_service()
            self.traj_service(self.traj_req)

            for i in range(self.num_periods):
                if rospy.is_shutdown():
                    break
                rospy.loginfo('Iteration %d/%d...', i+1, self.num_periods)
                self.exec_service()
            self.clear_service()
        except rospy.ServiceException as e:
            rospy.logerr('Could not execute trajectory: %s', str(e))

if __name__ == '__main__':
    rospy.init_node('sin_poses')
    try:
        sp = SinusoidalPoses()
        sp.execute()
    except rospy.ROSInterruptException:
        pass
