add_executable(map_compiler map_compiler.cpp)
target_compile_options(map_compiler PRIVATE -Wall -Wextra)
target_link_libraries(map_compiler PRIVATE glim_localization)

# lib/<package> is where `ros2 run` looks for executables.
install(TARGETS map_compiler RUNTIME DESTINATION lib/glim_localization)

add_executable(glim_localization_replay glim_localization_replay.cpp)
target_compile_options(glim_localization_replay PRIVATE -Wall -Wextra)
target_link_libraries(glim_localization_replay PRIVATE glim_localization nlohmann_json::nlohmann_json)
install(TARGETS glim_localization_replay RUNTIME DESTINATION lib/glim_localization)

# Evaluation utility: builds a GLIM map from a replay dataset (not a supported mapper)
add_executable(glim_localization_mapper glim_localization_mapper.cpp)
target_compile_options(glim_localization_mapper PRIVATE -Wall -Wextra)
target_link_libraries(glim_localization_mapper PRIVATE glim_localization nlohmann_json::nlohmann_json)
install(TARGETS glim_localization_mapper RUNTIME DESTINATION lib/glim_localization)
