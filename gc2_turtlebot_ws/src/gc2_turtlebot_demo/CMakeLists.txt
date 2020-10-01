cmake_minimum_required(VERSION 3.0.2)
project(gc2_turtlebot_demo)

find_package(catkin REQUIRED COMPONENTS
  kobuki_node
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gc2_turtlebot_demo
  CATKIN_DEPENDS kobuki_node roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES RELATIVE ./src *.cpp)

add_executable(gc2_turtlebot_demo ${SOURCES})
add_dependencies(gc2_turtlebot_demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(gc2_turtlebot_demo ${catkin_LIBRARIES})

install(TARGETS gc2_turtlebot_demo
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*")

#install(DIRECTORY launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
