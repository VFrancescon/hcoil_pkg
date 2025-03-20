cmake_minimum_required(VERSION 3.8)
project(hcoil_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hcoil_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

find_library(MID HCoilMiddlewareLib)
find_library(PSU DxkdpLib)
find_library(LINACT LinActuatorLib)
find_library(TMETER LakeshoreF71Lib)
find_library(SERIAL CppLinuxSerial)

set(MiddlewareLibs ${MID} ${PSU} ${LINACT} ${TMETER} ${SERIAL} Threads::Threads)
set(PSULibs ${PSU} ${SERIAL})
set(LinActLibs ${LINACT} ${SERIAL})
set(TmeterLibs ${TMETER} ${SERIAL})

option(BUILD_TESTING "Enable testing" OFF)

if(BUILD_TESTING)
  message("HCOIL_PKG: Building tests")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_psu_test
    test/psu_tests.cpp
    src/psu_node.cpp
  )

  ament_add_gtest(${PROJECT_NAME}_field_test
    test/field_tests.cpp
    src/field_node.cpp
  )

  target_include_directories(${PROJECT_NAME}_psu_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include
  )

  target_include_directories(${PROJECT_NAME}_field_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include
  )

  ament_target_dependencies(${PROJECT_NAME}_psu_test
    rclcpp
    hcoil_interfaces
    std_srvs
  )

  ament_target_dependencies(${PROJECT_NAME}_field_test
    rclcpp
    hcoil_interfaces
    std_srvs
  )

  target_link_libraries(${PROJECT_NAME}_psu_test ${PSULibs})

  target_compile_definitions(${PROJECT_NAME}_psu_test PRIVATE TESTING_EXCLUDE_MAIN=true)
  target_compile_definitions(${PROJECT_NAME}_field_test PRIVATE TESTING_EXCLUDE_MAIN=true)

  else()
  message("HCOIL_PKG: Not building tests")
  add_executable(psu_node src/psu_node.cpp)
  add_executable(field_node src/field_node.cpp)
  ament_target_dependencies(psu_node rclcpp hcoil_interfaces std_srvs)
  ament_target_dependencies(field_node rclcpp hcoil_interfaces std_srvs)
  target_link_libraries(psu_node ${PSULibs})
  target_link_libraries(field_node ${PSULibs})
  install(TARGETS
    psu_node

    DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS
    field_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
