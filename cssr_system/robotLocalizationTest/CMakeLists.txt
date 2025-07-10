# Add executables with the separated source files
add_executable(robotLocalizationTest src/robotLocalizationTestApplication.cpp src/robotLocalizationTestImplementation.cpp)
# Set properties for the executable
set_target_properties(robotLocalizationTest PROPERTIES OUTPUT_NAME robotLocalizationTest PREFIX "")
# Link the required libraries
target_link_libraries(robotLocalizationTest ${catkin_LIBRARIES} ${aruco_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${JSONCPP_LIBRARIES} yaml-cpp)
# Include the necessary directories
target_include_directories(robotLocalizationTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# Add dependencies
add_dependencies(robotLocalizationTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})