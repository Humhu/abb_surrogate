cmake_minimum_required(VERSION 2.8.3)
project( abb_surrogate )

set( CMAKE_CXX_FLAGS "-g -Wall -std=c++11" )
set( CMAKE_BUILD_TYPE Release )

find_package( catkin 	REQUIRED COMPONENTS
              			roscpp
              			argus_utils
	                    open_abb_driver )

catkin_package(
	INCLUDE_DIRS	include
	
	# LIBRARIES 		abb_surrogate
					
	CATKIN_DEPENDS 	roscpp
					argus_utils
)

include_directories( include
					 ${catkin_INCLUDE_DIRS} )

# add_library( abb_surrogate
			 # src/RobotInterface.cpp)
# add_dependencies( abb_surrogate ${abb_surrogate_gencfg} )
# target_link_libraries( abb_surrogate	${catkin_LIBRARIES} )

# add_executable( abb_node nodes/ABBNode.cpp )
# add_dependencies( abb_node ${PROJECT_NAME}_gencfg )
# target_link_libraries( abb_node abb_comm ${catkin_LIBRARIES} )

# install( TARGETS abb_surrogate 
	# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
