# add global_mapper library
add_library(global_mapper
  global_mapper.cc
  global_mapper_params.cc
  )

target_link_libraries(global_mapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

install(TARGETS global_mapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
