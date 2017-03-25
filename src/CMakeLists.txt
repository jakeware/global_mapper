# add node library
add_executable(global_mapper_node
  global_mapper_node.cc
)

# add node upstream dependencies
target_link_libraries(global_mapper_node
  ${catkin_LIBRARIES}
  global_mapper_ros
)

# install node
install(TARGETS global_mapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# add nodelet library
add_library(global_mapper_nodelet
  global_mapper_nodelet.cc
)

# add nodelet upstream dependencies
target_link_libraries(global_mapper_nodelet
  ${catkin_LIBRARIES}
  global_mapper_ros
)

# install nodelet
install(TARGETS global_mapper_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
