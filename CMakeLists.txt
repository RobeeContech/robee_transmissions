cmake_minimum_required(VERSION 3.8)
project(robee_transmissions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    transmission_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/scara_transmission_loader.cpp
  src/basic_transmission_loader.cpp
  src/piston_transmission_loader.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(transmission_interface  ros2_control_plugins.xml)
  #${PROJECT_NAME} ros2_control_plugins.xml)


# INSTALL
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)




if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(
    test_scara_transmission
    test/scara_transmission_test.cpp
  )
  target_include_directories(test_scara_transmission PUBLIC include hardware_interface transmission_interface)
  ament_target_dependencies(test_scara_transmission hardware_interface transmission_interface)


  ament_add_gmock(
    test_scara_transmission_loader
    test/scara_transmission_loader_test.cpp
  )
  target_include_directories(test_scara_transmission_loader PUBLIC include hardware_interface transmission_interface)
  ament_target_dependencies(test_scara_transmission_loader hardware_interface transmission_interface)



  ament_add_gmock(
    test_piston_transmission
    test/piston_transmission_test.cpp
  )
  target_include_directories(test_piston_transmission PUBLIC include hardware_interface transmission_interface)
  ament_target_dependencies(test_piston_transmission hardware_interface transmission_interface)


  ament_add_gmock(
    test_piston_transmission_loader
    test/piston_transmission_loader_test.cpp
  )
  target_include_directories(test_piston_transmission_loader PUBLIC include hardware_interface transmission_interface)
  ament_target_dependencies(test_piston_transmission_loader hardware_interface transmission_interface)

endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  hardware_interface
  transmission_interface
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
