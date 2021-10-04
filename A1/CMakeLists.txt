add_executable(julia julia_gpu.cu utilities.h utilities.cu)
target_link_libraries(julia PUBLIC glut book)
target_compile_features(julia PUBLIC cuda_std_14)

set_target_properties(julia PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)

add_custom_command(TARGET julia POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:glut> $<TARGET_FILE_DIR:julia>
    )