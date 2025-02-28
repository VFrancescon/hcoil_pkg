cmake_minimum_required(VERSION 3.8)
project(hcoil_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hcoil_interfaces REQUIRED)    
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

find_library(MID HCoilMiddlewareLib )
find_library(PSU  DxkdpLib )
find_library(LINACT LinActuatorLib )
find_library(TMETER LakeshoreF71Lib )
find_library(SERIAL CppLinuxSerial )
find_library( ASTAR a-star)

set(MiddlewareLibs ${MID} ${PSU} ${LINACT} ${TMETER} ${SERIAL} Threads::Threads)
set(PSULibs ${PSU} ${SERIAL})
set(LinActLibs ${LINACT} ${SERIAL})
set(TmeterLibs ${TMETER} ${SERIAL})



if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_psu_test 
    test/psu_tests.cpp
    src/psu_node.cpp
  )
  target_include_directories(${PROJECT_NAME}_psu_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include
  )
  ament_target_dependencies(${PROJECT_NAME}_psu_test
    rclcpp 
    hcoil_interfaces
  )
  target_link_libraries(${PROJECT_NAME}_psu_test ${PSULibs})
else()
  add_executable(psu_node src/psu_node.cpp)
  ament_target_dependencies(psu_node rclcpp hcoil_interfaces)
  target_link_libraries(psu_node ${PSULibs})
  install(TARGETS
    psu_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
