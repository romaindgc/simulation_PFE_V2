cmake_minimum_required(VERSION 3.8)
project(tpe_robot_mathlib)

set(tpe_robot_mathlib_MAJOR_VERSION 0)
set(tpe_robot_mathlib_MINOR_VERSION 0)
set(tpe_robot_mathlib_PATCH_VERSION 0)
set(tpe_robot_mathlib_VERSION
  ${tpe_robot_mathlib_MAJOR_VERSION}.${tpe_robot_mathlib_MINOR_VERSION}.${tpe_robot_mathlib_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(mathlib SHARED src/mathlib.cpp)
target_compile_features(mathlib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(mathlib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mathlib PRIVATE "TPE_ROBOT_MATHLIB_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mathlib
  EXPORT export_tpe_robot_mathlib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# export targets
set(export_targets ${export_targets};mathlib)
export(EXPORT export_tpe_robot_mathlib
  FILE "${PROJECT_BINARY_DIR}/export_tpe_robot_mathlib.cmake")

# Create the tpe_robot_mathlibConfig.cmake
set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
configure_file(tpe_robot_mathlibConfig.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/tpe_robot_mathlibConfig.cmake" @ONLY)

# Create the tpe_robot_mathlibConfigVersion.cmake
configure_file(tpe_robot_mathlibConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/tpe_robot_mathlibConfigVersion.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/tpe_robot_mathlibConfig.cmake"
  "${PROJECT_BINARY_DIR}/tpe_robot_mathlibConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev)
install(EXPORT export_tpe_robot_mathlib
  DESTINATION "share/${PROJECT_NAME}/cmake"
  FILE export_tpe_robot_mathlib.cmake
  COMPONENT dev)
