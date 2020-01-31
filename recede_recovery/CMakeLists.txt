cmake_minimum_required(VERSION 2.8.3)
project(recede_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            costmap_2d
            nav_core
            pluginlib
            base_local_planner
        )

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES recede_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(recede_recovery src/recede_recovery.cpp)
add_dependencies(recede_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recede_recovery ${catkin_LIBRARIES})

install(TARGETS recede_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES recede_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
