# add global_mapper library
add_library(global_mapper
  global_mapper.cc
  )

target_link_libraries(global_mapper
  ${catkin_LIBRARIES}
  )

# add nodelet downstream dependencies
add_dependencies(global_mapper
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

install(TARGETS global_mapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
