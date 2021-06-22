cmake_minimum_required(VERSION 3.0.2)
project(testforopencv)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs OpenCV cv_bridge image_transport)

catkin_package(
	CATKIN_DEPENDS roscpp rospy 

)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)
add_executable(sensortocv src/twoColorDetection.cpp)
add_dependencies(sensortocv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensortocv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(traindata src/getTrainingData.cpp)
add_dependencies(traindata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traindata ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})