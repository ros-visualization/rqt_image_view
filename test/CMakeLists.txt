find_package(ament_cmake_gtest REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

ament_add_gtest(test_linear_to_srgb test_linear_to_srgb.cpp)
target_include_directories(test_linear_to_srgb PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)
target_link_libraries(test_linear_to_srgb
  opencv_core
)

# Note: ament_cmake_gtest links its own dependencies onto the target with
# the plain-form target_link_libraries; CMake then requires us to use the
# same form here. Visibility keyword (PRIVATE) would error out.
foreach(suite pixel_format pixel_mapping)
  ament_add_gtest(test_${suite} test_${suite}.cpp)
  target_link_libraries(test_${suite} ${PROJECT_NAME})
endforeach()
