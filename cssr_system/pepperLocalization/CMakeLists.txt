# Add the new executable with the separated source files
add_executable(pepperLocalization src/pepperLocalizationImplementation.cpp)
# Set properties for the executable
set_target_properties(pepperLocalization PROPERTIES OUTPUT_NAME pepperLocalization PREFIX "")
# Link the required libraries
target_link_libraries(pepperLocalization ${catkin_LIBRARIES} ${aruco_INCLUDE_DIRS} ${OpenCV_LIBRARIES} yaml-cpp)
# Include the necessary directories
target_include_directories(pepperLocalization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# Add dependencies
add_dependencies(pepperLocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})