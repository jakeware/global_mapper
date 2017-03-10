# add global_mapper_ros library
add_library(global_mapper_ros
  global_mapper_ros.cc
  )

target_link_libraries(global_mapper_ros
  ${catkin_LIBRARIES}
  )

# add nodelet downstream dependencies
add_dependencies(global_mapper_ros
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  global_mapper
  )

install(TARGETS global_mapper_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
