# add global_mapper_ros library
add_library(global_mapper_ros
  global_mapper_ros.cc
  )

target_link_libraries(global_mapper_ros
  global_mapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

install(TARGETS global_mapper_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
