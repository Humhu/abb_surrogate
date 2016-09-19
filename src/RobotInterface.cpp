#include "abb_surrogate/RobotInterface.h"

#include "argus_utils/utils/ParamUtils.h"

#include "open_abb_driver/SetSpeed.h"
#include "open_abb_driver/SetCartesian.h"

#include <ros/service.h>

#define DEG_2_RAD (0.0174533)

namespace argus
{

RobotInterface::RobotInterface( ros::NodeHandle& nh, ros::NodeHandle& ph ) 
{
	std::string speedTopic, poseTopic;
	GetParamRequired( ph, "set_speed_topic", speedTopic );
	GetParamRequired( ph, "set_pose_topic", poseTopic );
	ros::service::waitForService( speedTopic );
	ros::service::waitForService( poseTopic );
	_speedClient = nh.serviceClient<open_abb_driver::SetSpeed>( speedTopic, true );
	_poseClient = nh.serviceClient<open_abb_driver::SetCartesian>( poseTopic, true );
}

bool RobotInterface::SetSpeed( double speedLin, double speedAng )
{
	open_abb_driver::SetSpeed srv;
	srv.request.tcp = speedLin;
	srv.request.ori = speedAng * DEG_2_RAD;
	return _speedClient.call( srv );
}

bool RobotInterface::SetReferencePose( const PoseSE3& pose )
{
	open_abb_driver::SetCartesian srv;
	Translation3Type trans = pose.GetTranslation();
	QuaternionType quat = pose.GetQuaternion();
	srv.x = trans.x();
	srv.y = trans.y();
	srv.z = trans.z();
	srv.qw = quat.w();
	srv.qx = quat.x();
	srv.qy = quat.y();
	srv.qz = quat.z();
	return _poseClient.call( srv );
}

}