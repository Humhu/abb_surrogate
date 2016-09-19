#pragma once

#include <ros/ros.h>
#include "argus_utils/geometry/PoseSE3.h"

namespace argus
{

// Provides an interface similar to a ground robot around the ABB arm
class RobotInterface
{
public:

	RobotInterface( ros::NodeHandle& nh, ros::NodeHandle& ph );

	bool SetSpeed( double speedLin, double speedAng );
	bool SetReferencePose( const PoseSE3& pose );

private:

	ros::ServiceClient _speedClient;
	ros::ServiceClient _poseClient;
};

}