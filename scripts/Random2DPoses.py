#!/usr/bin/env python

import rospy, random
import numpy as np
# from open_abb_driver.srv import SetSpeed, SetSpeedRequest
from open_abb_driver.srv import SetCartesianLinear, SetCartesianLinearRequest

class Random2DPoses:

    def __init__( self ):

        # set_speed_topic = rospy.get_param( '~speed_service' )
        set_pose_topic = rospy.get_param( '~pose_service' )
        # rospy.wait_for_service( set_speed_topic )
        rospy.wait_for_service( set_pose_topic )
        # self.speed_service = rospy.ServiceProxy( set_speed_topic, SetSpeed )
        self.pose_service = rospy.ServiceProxy( set_pose_topic, SetCartesianLinear )

        self.x_lims = rospy.get_param( '~x_lims' )
        if type( self.x_lims ) is not list or len( self.x_lims ) != 2:
            raise ValueError( 'Random2DPoses: x_lims must be [min,max]' )

        self.y_lims = rospy.get_param( '~y_lims' )
        if type( self.y_lims ) is not list or len( self.y_lims ) != 2:
            raise ValueError( 'Random2DPoses: y_lims must be [min,max]' )

        self.z_value = rospy.get_param( '~z_value' )
        self.ori = rospy.get_param( '~orientation' )
        self.step_time = rospy.get_param( '~step_time' )
        self.num_waypoints = rospy.get_param( '~num_waypoints' )

        step_rate = rospy.get_param( '~step_rate' )
        self.timer = rospy.Timer( rospy.Duration( 1.0/step_rate ), self.TimerCallback )

    def TimerCallback( self, event ):

        x = random.uniform( self.x_lims[0], self.x_lims[1] )
        y = random.uniform( self.y_lims[0], self.y_lims[1] )
        
        rospy.loginfo( 'Traversing to x: %f, y: %f', x, y )

        req = SetCartesianLinearRequest()
        req.pose.position.x = x;
        req.pose.position.y = y;
        req.pose.position.z = self.z_value
        req.pose.orientation.w = self.ori[0]
        req.pose.orientation.x = self.ori[1]
        req.pose.orientation.y = self.ori[2]
        req.pose.orientation.z = self.ori[3]
        req.duration = self.step_time
        req.num_waypoints = self.num_waypoints

        try:
            self.pose_service( req )
        except rospy.ServiceException as e:
            ros.logerror( 'Could not query critique: ' + str(e) )

if __name__=='__main__':
    rospy.init_node( 'random_2d_poses' )
    try:
        r2dp = Random2DPoses()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

