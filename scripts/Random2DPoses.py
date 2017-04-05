#!/usr/bin/env python

import rospy, random
import numpy as np

# from open_abb_driver.srv import SetSpeed, SetSpeedRequest
from open_abb_driver.srv import SetCartesianLinear, SetCartesianLinearRequest
from percepto_msgs.msg import EpisodeBreak

class Random2DPoses:

    def __init__( self ):

        set_pose_topic = rospy.get_param( '~pose_service' )
        rospy.wait_for_service( set_pose_topic )
        self.pose_service = rospy.ServiceProxy( set_pose_topic, SetCartesianLinear )

        if rospy.has_param( '~seed' ):
            seed = rospy.get_param( '~seed' )
            random.seed( seed )
            rospy.loginfo('Setting seed to %d', seed)

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

        dwell_time = rospy.get_param( '~dwell_time' )
        self.loop_wait = rospy.Duration( dwell_time )

        self.iter_counter = 0
        self.iters_to_run = rospy.get_param( '~iterations', -1 )

        self.break_pub = rospy.Publisher('~breaks', EpisodeBreak, queue_size=10)

    def Execute( self ):
        while not rospy.is_shutdown():
            if self.iters_to_run != -1 and self.iter_counter >= self.iters_to_run:
                break

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
                rospy.logerr( 'Could not move to pose: ' + str(e) )

            bmsg = EpisodeBreak()
            bmsg.break_time = rospy.Time.now()
            self.break_pub.publish(bmsg)

            rospy.sleep( self.loop_wait )
            self.iter_counter += 1

if __name__=='__main__':
    rospy.init_node( 'random_2d_poses' )
    try:
        r2dp = Random2DPoses()
        r2dp.Execute()
    except rospy.ROSInterruptException:
        pass

