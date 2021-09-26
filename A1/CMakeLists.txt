cmake_minimum_required(VERSION 3.20)
project(julia CUDA)

add_subdirectory(glut)
add_subdirectory(common)

add_executable(julia julia_gpu.cu utilities.h utilities.cu)
target_link_libraries(julia PUBLIC glut book)
target_compile_features(julia PUBLIC cuda_std_14)

set_target_properties(julia PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
