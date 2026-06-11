cmake_minimum_required(VERSION 3.8)
project(beckhoff_ads_hardware_interface)

include(ExternalProject)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

set(ADS_EXTERNAL_PROJECT_NAME ${PROJECT_NAME}_ads_external_project)
set(ADS_GIT_REPOSITORY "https://github.com/Beckhoff/ADS.git")
set(ADS_GIT_TAG "e85ef696447a77039a4c3710d7573c8df32c0d58")

set(ADS_EXTERNAL_PROJECT_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ads_external_build)
set(ADS_EXTERNAL_PROJECT_SOURCE_DIR ${ADS_EXTERNAL_PROJECT_PREFIX}/src/Ads)
set(ADS_EXTERNAL_PROJECT_BINARY_DIR ${ADS_EXTERNAL_PROJECT_PREFIX}/build/Ads)
set(ADS_EXTERNAL_PROJECT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/ads_install)
file(MAKE_DIRECTORY
  ${ADS_EXTERNAL_PROJECT_INSTALL_DIR}/include
  ${ADS_EXTERNAL_PROJECT_INSTALL_DIR}/lib
)

if(CMAKE_BUILD_TYPE)
  set(ADS_EXTERNAL_PROJECT_BUILD_TYPE ${CMAKE_BUILD_TYPE})
else()
  set(ADS_EXTERNAL_PROJECT_BUILD_TYPE RelWithDebInfo)
endif()

set(ADS_EXTERNAL_PROJECT_BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR>)
set(ADS_EXTERNAL_PROJECT_INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>)
if(ADS_EXTERNAL_PROJECT_BUILD_TYPE)
  list(APPEND ADS_EXTERNAL_PROJECT_BUILD_COMMAND --config ${ADS_EXTERNAL_PROJECT_BUILD_TYPE})
  list(APPEND ADS_EXTERNAL_PROJECT_INSTALL_COMMAND --config ${ADS_EXTERNAL_PROJECT_BUILD_TYPE})
endif()

ExternalProject_Add(${ADS_EXTERNAL_PROJECT_NAME}
  GIT_REPOSITORY    ${ADS_GIT_REPOSITORY}
  GIT_TAG           ${ADS_GIT_TAG}

  PREFIX            ${ADS_EXTERNAL_PROJECT_PREFIX}
  SOURCE_DIR        ${ADS_EXTERNAL_PROJECT_SOURCE_DIR}
  BINARY_DIR        ${ADS_EXTERNAL_PROJECT_BINARY_DIR}
  INSTALL_DIR       ${ADS_EXTERNAL_PROJECT_INSTALL_DIR}

  UPDATE_COMMAND    ""

  PATCH_COMMAND     ${CMAKE_COMMAND}
                    -DADS_SOURCE_DIR=<SOURCE_DIR>
                    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_ads.cmake

  CONFIGURE_COMMAND ${CMAKE_COMMAND} -S <SOURCE_DIR> -B <BINARY_DIR>
                    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
                    -DCMAKE_BUILD_TYPE=${ADS_EXTERNAL_PROJECT_BUILD_TYPE}
                    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
                    -DBUILD_SHARED_LIBS=OFF

  BUILD_COMMAND     ${CMAKE_COMMAND}
                    -DADS_SOURCE_DIR=<SOURCE_DIR>
                    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_ads.cmake
                    COMMAND ${ADS_EXTERNAL_PROJECT_BUILD_COMMAND}
  BUILD_ALWAYS      TRUE

  INSTALL_COMMAND   ${ADS_EXTERNAL_PROJECT_INSTALL_COMMAND}

  LOG_CONFIGURE     YES
  LOG_BUILD         YES
  LOG_INSTALL       YES
)

add_library(beckhoff_ads_patched STATIC IMPORTED GLOBAL)
set_target_properties(beckhoff_ads_patched
  PROPERTIES
    IMPORTED_LOCATION ${ADS_EXTERNAL_PROJECT_INSTALL_DIR}/lib/libads.a
    INTERFACE_INCLUDE_DIRECTORIES ${ADS_EXTERNAL_PROJECT_INSTALL_DIR}/include
    INTERFACE_LINK_LIBRARIES Threads::Threads
)
add_dependencies(beckhoff_ads_patched ${ADS_EXTERNAL_PROJECT_NAME})

add_library(
  beckhoff_ads_hardware_interface
  SHARED
  src/beckhoff_ads_hardware_interface.cpp
)

target_include_directories(
  beckhoff_ads_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ADS_EXTERNAL_PROJECT_INSTALL_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(beckhoff_ads_hardware_interface PUBLIC
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  beckhoff_ads_patched
)
add_dependencies(beckhoff_ads_hardware_interface ${ADS_EXTERNAL_PROJECT_NAME})

# prevent pluginlib from using boost
target_compile_definitions(beckhoff_ads_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface beckhoff_ads_hardware_interface.xml)

install(
  TARGETS
  beckhoff_ads_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY ${ADS_EXTERNAL_PROJECT_INSTALL_DIR}/include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  beckhoff_ads_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
