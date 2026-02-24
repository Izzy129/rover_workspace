cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Source of truth: urdf_models/rigid_body (relative to this package's source dir)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../urdf_models/rigid_body/robot.urdf
  DESTINATION share/${PROJECT_NAME}/urdf/
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../urdf_models/rigid_body/assets
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
