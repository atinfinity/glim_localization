find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(test_version test_version.cpp)
target_link_libraries(test_version PRIVATE glim_localization GTest::gtest_main)
gtest_discover_tests(test_version)

add_executable(test_process_claim test_process_claim.cpp)
target_link_libraries(test_process_claim PRIVATE glim_localization GTest::gtest_main)
gtest_discover_tests(test_process_claim)

add_executable(test_config test_config.cpp)
target_compile_definitions(test_config PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_config PRIVATE nlohmann_json::nlohmann_json GTest::gtest_main)
gtest_discover_tests(test_config)

# Synthetic data shared by tests. Not installed.
add_library(glim_localization_synthetic STATIC
  synthetic/synthetic_map.cpp
  synthetic/synthetic_replay.cpp
  synthetic/synthetic_sensor.cpp
)
target_include_directories(glim_localization_synthetic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(glim_localization_synthetic PUBLIC glim_localization)

add_executable(test_synthetic_map test_synthetic_map.cpp)
target_link_libraries(test_synthetic_map PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_synthetic_map)

add_executable(test_synthetic_sensor test_synthetic_sensor.cpp)
target_compile_definitions(test_synthetic_sensor PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_synthetic_sensor PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_synthetic_sensor PROPERTIES TIMEOUT 120)

add_executable(test_map_factor test_map_factor.cpp)
target_link_libraries(test_map_factor PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_map_factor PROPERTIES TIMEOUT 120)

add_executable(test_quality_monitor test_quality_monitor.cpp)
target_link_libraries(test_quality_monitor PRIVATE glim_localization GTest::gtest_main)
gtest_discover_tests(test_quality_monitor)

add_executable(test_localization_odometry test_localization_odometry.cpp)
target_compile_definitions(test_localization_odometry PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_localization_odometry PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_localization_odometry PROPERTIES TIMEOUT 120)

add_executable(test_wheel_odometry test_wheel_odometry.cpp)
target_compile_definitions(test_wheel_odometry PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_wheel_odometry PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_wheel_odometry PROPERTIES TIMEOUT 300)

add_executable(test_relocalization test_relocalization.cpp)
target_link_libraries(test_relocalization PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_relocalization PROPERTIES TIMEOUT 180)

add_executable(test_init_yaw_search test_init_yaw_search.cpp)
target_compile_definitions(test_init_yaw_search PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_init_yaw_search PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_init_yaw_search PROPERTIES TIMEOUT 300)

add_executable(test_plugin test_plugin.cpp)
target_compile_definitions(test_plugin PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
  GLIM_LOCALIZATION_PLUGIN_PATH="$<TARGET_FILE:odometry_localization_cpu>"
)
target_link_libraries(test_plugin PRIVATE
  glim_localization_synthetic
  nlohmann_json::nlohmann_json
  GTest::gtest_main
)
add_dependencies(test_plugin odometry_localization_cpu)
gtest_discover_tests(test_plugin PROPERTIES TIMEOUT 120)

add_executable(test_map_io test_map_io.cpp)
target_link_libraries(test_map_io PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_map_io)

add_executable(test_map_manager test_map_manager.cpp)
target_link_libraries(test_map_manager PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_map_manager)

add_executable(test_tile_residency test_tile_residency.cpp)
target_link_libraries(test_tile_residency PRIVATE glim_localization GTest::gtest_main)
gtest_discover_tests(test_tile_residency)

add_executable(test_tile_prefetch test_tile_prefetch.cpp)
target_link_libraries(test_tile_prefetch PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_tile_prefetch)

find_package(Threads REQUIRED)
add_executable(test_tile_streamer test_tile_streamer.cpp)
target_link_libraries(test_tile_streamer PRIVATE glim_localization Threads::Threads GTest::gtest_main)
gtest_discover_tests(test_tile_streamer PROPERTIES TIMEOUT 60)

add_executable(test_pcd_map test_pcd_map.cpp)
target_link_libraries(test_pcd_map PRIVATE glim_localization_synthetic nlohmann_json::nlohmann_json GTest::gtest_main)
gtest_discover_tests(test_pcd_map)

add_executable(test_ply_reader test_ply_reader.cpp)
target_link_libraries(test_ply_reader PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_ply_reader)

add_executable(test_occupancy_grid test_occupancy_grid.cpp)
target_link_libraries(test_occupancy_grid PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_occupancy_grid)

# GPU smoke test (BUILD_WITH_CUDA=ON only). Skips without a visible GPU; ctest reports the skip.
if(GLIM_LOCALIZATION_WITH_CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_executable(test_gpu_smoke test_gpu_smoke.cpp)
  target_link_libraries(test_gpu_smoke PRIVATE glim_localization CUDA::cudart GTest::gtest_main)
  gtest_discover_tests(test_gpu_smoke PROPERTIES
    TIMEOUT 120
    SKIP_REGULAR_EXPRESSION "\\[  SKIPPED \\]"
  )

  add_executable(test_localization_odometry_gpu test_localization_odometry_gpu.cpp)
  target_compile_definitions(test_localization_odometry_gpu PRIVATE
    GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
    GLIM_LOCALIZATION_GPU_PLUGIN_PATH="$<TARGET_FILE:odometry_localization_gpu>"
  )
  target_link_libraries(test_localization_odometry_gpu PRIVATE
    glim_localization_synthetic
    nlohmann_json::nlohmann_json
    CUDA::cudart
    GTest::gtest_main
  )
  add_dependencies(test_localization_odometry_gpu odometry_localization_gpu)
  gtest_discover_tests(test_localization_odometry_gpu PROPERTIES
    TIMEOUT 300
    SKIP_REGULAR_EXPRESSION "\\[  SKIPPED \\]"
  )
endif()

add_executable(test_localization_backend test_localization_backend.cpp)
target_compile_definitions(test_localization_backend PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_localization_backend PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_localization_backend PROPERTIES TIMEOUT 120)

if(TARGET map_compiler)
  add_executable(test_map_compiler test_map_compiler.cpp)
  target_compile_definitions(test_map_compiler PRIVATE MAP_COMPILER_PATH="$<TARGET_FILE:map_compiler>")
  target_link_libraries(test_map_compiler PRIVATE
    glim_localization_synthetic
    nlohmann_json::nlohmann_json
    GTest::gtest_main
  )
  add_dependencies(test_map_compiler map_compiler)
  gtest_discover_tests(test_map_compiler)
endif()

add_executable(test_replay_dataset test_replay_dataset.cpp)
target_link_libraries(test_replay_dataset PRIVATE glim_localization nlohmann_json::nlohmann_json GTest::gtest_main)
gtest_discover_tests(test_replay_dataset)

add_executable(test_synthetic_replay test_synthetic_replay.cpp)
target_link_libraries(test_synthetic_replay PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_synthetic_replay PROPERTIES TIMEOUT 120)

# Writes a synthetic map and replay dataset for the end-to-end check. Not installed.
add_executable(synthetic_replay_dataset synthetic_replay_dataset.cpp)
target_link_libraries(synthetic_replay_dataset PRIVATE glim_localization_synthetic)

add_executable(test_replay_runner test_replay_runner.cpp)
target_compile_definitions(test_replay_runner PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_replay_runner PRIVATE glim_localization_synthetic nlohmann_json::nlohmann_json GTest::gtest_main)
gtest_discover_tests(test_replay_runner PROPERTIES TIMEOUT 180)

add_executable(test_replay_deskew test_replay_deskew.cpp)
target_compile_definitions(test_replay_deskew PRIVATE
  GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)
target_link_libraries(test_replay_deskew PRIVATE glim_localization_synthetic GTest::gtest_main)
gtest_discover_tests(test_replay_deskew PROPERTIES TIMEOUT 180)

if(TARGET glim_localization_replay)
  add_executable(test_replay_tool test_replay_tool.cpp)
  target_compile_definitions(test_replay_tool PRIVATE
    REPLAY_TOOL_PATH="$<TARGET_FILE:glim_localization_replay>"
    GLIM_LOCALIZATION_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
  )
  target_link_libraries(test_replay_tool PRIVATE glim_localization_synthetic nlohmann_json::nlohmann_json GTest::gtest_main)
  add_dependencies(test_replay_tool glim_localization_replay)
  gtest_discover_tests(test_replay_tool PROPERTIES TIMEOUT 180)
endif()

if(TARGET glim_localization_mapper)
  add_executable(test_mapper_tool test_mapper_tool.cpp)
  target_compile_definitions(test_mapper_tool PRIVATE
    MAPPER_TOOL_PATH="$<TARGET_FILE:glim_localization_mapper>"
    MAPPING_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config/mapping_cpu"
  )
  target_link_libraries(test_mapper_tool PRIVATE glim_localization_synthetic nlohmann_json::nlohmann_json GTest::gtest_main)
  add_dependencies(test_mapper_tool glim_localization_mapper)
  gtest_discover_tests(test_mapper_tool PROPERTIES TIMEOUT 300)
endif()
