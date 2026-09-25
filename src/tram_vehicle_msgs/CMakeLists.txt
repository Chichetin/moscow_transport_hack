cmake_minimum_required(VERSION 3.14)
project(tram_vehicle_msgs_vendor NONE)

find_package(ament_cmake REQUIRED)

# Jury's tram_vehicle_msgs (same workspace or underlay) wins; ours is built only when there is
# none. Nested in this package, so colcon never sees two tram_vehicle_msgs (D-041, #40).
find_package(tram_vehicle_msgs QUIET)
if(tram_vehicle_msgs_FOUND)
  message(STATUS "tram_vehicle_msgs found in ${tram_vehicle_msgs_DIR}: vendored copy not built")
else()
  message(STATUS "tram_vehicle_msgs not found: building vendored copy")
  set(_src "${CMAKE_CURRENT_BINARY_DIR}/tram_vehicle_msgs")
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/vendor/package.xml" "${CMAKE_CURRENT_SOURCE_DIR}/vendor/CMakeLists.txt"
       "${CMAKE_CURRENT_SOURCE_DIR}/msg" DESTINATION "${_src}")
  # colcon sources only this package's hooks, not the nested one's: PYTHONPATH for the
  # generated Python module is registered here, with the same install dir for both builds
  find_package(ament_cmake_python REQUIRED)
  _ament_cmake_python_get_python_install_dir()
  _ament_cmake_python_register_environment_hook()
  include(ExternalProject)
  ExternalProject_Add(tram_vehicle_msgs_vendored
    SOURCE_DIR "${_src}"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/tram_vehicle_msgs-build"
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DBUILD_TESTING=OFF
               -DPYTHON_INSTALL_DIR=${PYTHON_INSTALL_DIR}
    )
endif()

ament_package()
