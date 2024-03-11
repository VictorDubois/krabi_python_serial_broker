cmake_minimum_required(VERSION 3.5)
project(krabi_python_serial_broker)

find_package(ament_cmake REQUIRED)

#install(
#  DIRECTORY models
#  DESTINATION share/${PROJECT_NAME}
#)
#install(DIRECTORY
#  worlds
#  DESTINATION share/${PROJECT_NAME}/
#)
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
    DIRECTORY scripts/
    DESTINATION lib/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
    FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

#install(DIRECTORY params
#  DESTINATION share/${PROJECT_NAME})

find_package(rclpy REQUIRED)
find_package(krabi_msgs REQUIRED)

# Install Python scripts
#ament_python_install_package()

# Export dependencies
ament_export_dependencies(rclpy)

#ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/env_variables.bash")
ament_package()