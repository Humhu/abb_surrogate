cmake_minimum_required(VERSION 2.8.3)
project( abb_surrogate )

find_package( catkin 	REQUIRED 
	COMPONENTS			rospy
              			argus_utils
	                    open_abb_driver
	                    message_generation)

catkin_python_setup()

add_service_files( FILES SetSpeed.srv )
generate_messages()

catkin_package(
	CATKIN_DEPENDS 	rospy
					argus_utils
					message_generation
)


include_directories( include
					 ${catkin_INCLUDE_DIRS} )
