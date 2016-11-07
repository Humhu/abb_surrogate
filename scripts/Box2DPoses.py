#!/usr/bin/env python

import rospy, random
import numpy as np
# from open_abb_driver.srv import SetSpeed, SetSpeedRequest
from open_abb_driver.srv import SetCartesianLinear, SetCartesianLinearRequest
from paraset.srv import StartEvaluation

class Box2DPoses:

    def __init__( self ):
        # set_speed_topic = rospy.get_param( '~speed_service' )
        set_pose_topic = rospy.get_param( '~pose_service' )
        # rospy.wait_for_service( set_speed_topic )
        rospy.wait_for_service( set_pose_topic )
        # self.speed_service = rospy.ServiceProxy( set_speed_topic, SetSpeed )
        self.pose_service = rospy.ServiceProxy( set_pose_topic, SetCartesianLinear )

        self.x_lims = rospy.get_param( '~x_lims' )
        if type( self.x_lims ) is not list or len( self.x_lims ) != 2:
            raise ValueError( 'Box2DPoses: x_lims must be [min,max]' )

        self.y_lims = rospy.get_param( '~y_lims' )
        if type( self.y_lims ) is not list or len( self.y_lims ) != 2:
            raise ValueError( 'Box2DPoses: y_lims must be [min,max]' )

        self.z_value = rospy.get_param( '~z_value' )
        self.ori = rospy.get_param( '~orientation' )
        self.step_time = rospy.get_param( '~step_time' )
        self.num_waypoints = rospy.get_param( '~num_waypoints' )
        self.num_loops = rospy.get_param( '~num_loops' )

        dwell_time = rospy.get_param( '~dwell_time' )
        self.loop_wait = rospy.Duration( dwell_time )

        self.waypoints = [ (self.x_lims[0], self.y_lims[0]),
                           (self.x_lims[0], self.y_lims[1]),
                           (self.x_lims[1], self.y_lims[1]),
                           (self.x_lims[1], self.y_lims[0]) ]

        self.evaluation_service = rospy.Service( '~start_trajectory', StartEvaluation, 
                                                 self.EvaluationCallback )

    def EvaluationCallback( self, srv ):
        for i in range(self.num_loops):
            for (x,y) in self.waypoints:
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
                    rospy.logerror( 'Could not query critique: ' + str(e) )
                    return None

                rospy.sleep( self.loop_wait )
        return []

if __name__=='__main__':
    rospy.init_node( 'random_2d_poses' )
    try:
        r2dp = Box2DPoses()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

