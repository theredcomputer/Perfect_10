###############################################
# apps/swing
file(GLOB swing_srcs "*.cpp")
file(GLOB swing_hdrs "*.h")
add_executable(swing ${swing_srcs} ${swing_hdrs})
target_link_libraries(swing dart)
set_target_properties(swing PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
