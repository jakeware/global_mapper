# add nodelet library
add_library(global_mapper_nodelet
  global_mapper_nodelet.cc
)

# add nodelet upstream dependencies
target_link_libraries(global_mapper_nodelet
  ${catkin_LIBRARIES}
  global_mapper_ros
)

# add nodelet downstream dependencies
add_dependencies(global_mapper_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  global_mapper_ros
  )

# install nodelet
install(TARGETS global_mapper_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
