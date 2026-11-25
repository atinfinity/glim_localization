cmake_minimum_required(VERSION 3.22)
project(glim_localization VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD_WITH_CUDA "Build GPU modules (requires a CUDA-enabled gtsam_points)" OFF)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_TOOLS "Build command line tools" ON)

find_package(ament_cmake QUIET)
option(GLIM_LOCALIZATION_USE_AMENT "Export this package with ament" ${ament_cmake_FOUND})

# glim-config.cmake sets BUILD_WITH_CUDA in the caller's scope, so keep our value.
set(GLIM_LOCALIZATION_WITH_CUDA ${BUILD_WITH_CUDA})

find_package(gtsam_points REQUIRED)
find_package(glim REQUIRED)
find_package(nlohmann_json REQUIRED)

if(GLIM_LOCALIZATION_WITH_CUDA AND NOT GTSAM_POINTS_USE_CUDA)
  message(FATAL_ERROR "BUILD_WITH_CUDA=ON requires gtsam_points built with CUDA")
endif()

add_library(glim_localization SHARED
  src/glim_localization/version.cpp
  src/glim_localization/map/map_io.cpp
  src/glim_localization/map/map_manager.cpp
  src/glim_localization/map/occupancy_grid.cpp
  src/glim_localization/map/pcd_map.cpp
  src/glim_localization/map/ply_reader.cpp
  src/glim_localization/odometry/map_factor_builder.cpp
  src/glim_localization/odometry/localization_backend.cpp
  src/glim_localization/odometry/map_initial_state_estimation.cpp
  src/glim_localization/odometry/odometry_estimation_localization.cpp
  src/glim_localization/odometry/odometry_estimation_localization_cpu.cpp
  src/glim_localization/odometry/quality_monitor.cpp
  src/glim_localization/odometry/weighted_factor.cpp
  src/glim_localization/odometry/wheel_odometry.cpp
  src/glim_localization/relocalization/candidate_search.cpp
  src/glim_localization/relocalization/pose_candidates.cpp
  src/glim_localization/replay/replay_dataset.cpp
  src/glim_localization/replay/replay_runner.cpp
)
target_compile_definitions(glim_localization PRIVATE
  GLIM_LOCALIZATION_VERSION="${PROJECT_VERSION}"
)
if(GLIM_LOCALIZATION_WITH_CUDA)
  find_package(CUDAToolkit REQUIRED)
  target_sources(glim_localization PRIVATE
    src/glim_localization/map/gpu_tile_cache.cpp
    src/glim_localization/odometry/gpu_linearization_hook.cpp
    src/glim_localization/odometry/odometry_estimation_localization_gpu.cpp
    src/glim_localization/odometry/weighted_vgicp_factor_gpu.cpp
  )
  target_compile_definitions(glim_localization PRIVATE GLIM_LOCALIZATION_WITH_CUDA)
  target_link_libraries(glim_localization PRIVATE CUDA::cudart)
endif()
target_compile_options(glim_localization PRIVATE -Wall -Wextra)
target_include_directories(glim_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(glim_localization PUBLIC
  glim::glim
  gtsam_points::gtsam_points
)
target_link_libraries(glim_localization PRIVATE nlohmann_json::nlohmann_json)
add_library(glim_localization::glim_localization ALIAS glim_localization)

# GLIM loads this by so_name with dlopen, so the file name is fixed on every platform.
add_library(odometry_localization_cpu MODULE
  src/glim_localization/odometry/odometry_estimation_localization_cpu_create.cpp
)
target_compile_options(odometry_localization_cpu PRIVATE -Wall -Wextra)
target_link_libraries(odometry_localization_cpu PRIVATE glim_localization)
set_target_properties(odometry_localization_cpu PROPERTIES
  PREFIX "lib"
  SUFFIX ".so"
  INSTALL_RPATH "$ORIGIN"
)

if(GLIM_LOCALIZATION_WITH_CUDA)
  add_library(odometry_localization_gpu MODULE
    src/glim_localization/odometry/odometry_estimation_localization_gpu_create.cpp
  )
  target_compile_options(odometry_localization_gpu PRIVATE -Wall -Wextra)
  target_link_libraries(odometry_localization_gpu PRIVATE glim_localization)
  set_target_properties(odometry_localization_gpu PROPERTIES
    PREFIX "lib"
    SUFFIX ".so"
    INSTALL_RPATH "$ORIGIN"
  )
  install(TARGETS odometry_localization_gpu LIBRARY DESTINATION lib)
endif()

if(BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(GLIM_LOCALIZATION_WITH_CUDA)
  install(DIRECTORY include/ DESTINATION include)
else()
  # The GPU headers need the CUDA build of gtsam_points
  install(DIRECTORY include/ DESTINATION include PATTERN "*gpu*" EXCLUDE)
endif()
install(DIRECTORY config/ DESTINATION share/glim_localization/config)
install(TARGETS glim_localization
  EXPORT glim_localization-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS odometry_localization_cpu LIBRARY DESTINATION lib)

if(GLIM_LOCALIZATION_USE_AMENT)
  ament_export_targets(glim_localization-targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(glim gtsam_points)
  ament_package()
else()
  include(CMakePackageConfigHelpers)
  set(config_install_dir lib/cmake/glim_localization)
  install(EXPORT glim_localization-targets
    NAMESPACE glim_localization::
    DESTINATION ${config_install_dir}
  )
  configure_package_config_file(
    cmake/glim_localization-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/glim_localization-config.cmake
    INSTALL_DESTINATION ${config_install_dir}
  )
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/glim_localization-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
  )
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/glim_localization-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/glim_localization-config-version.cmake
    DESTINATION ${config_install_dir}
  )
endif()
