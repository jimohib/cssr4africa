# Add the new executable with the separated source files
add_executable(arucoLocalization src/arucoLocalizationImplementation.cpp)
# Link the required libraries
target_link_libraries(arucoLocalization ${catkin_LIBRARIES} ${aruco_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
# Include the necessary directories
target_include_directories(arucoLocalization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# Add dependencies
add_dependencies(arucoLocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Marker Visualizer Node
# Add the new executable with the separated source files
add_executable(markerVisualizer src/markerVisualizer.cpp)
# Set properties for the executable
set_target_properties(markerVisualizer PROPERTIES OUTPUT_NAME markerVisualizer PREFIX "")
# Link the required libraries
target_link_libraries(markerVisualizer ${catkin_LIBRARIES} ${aruco_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
# Include the necessary directories
target_include_directories(markerVisualizer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# Add dependencies
add_dependencies(markerVisualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})