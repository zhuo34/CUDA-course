add_executable(ray ray.cu)
target_link_libraries(ray PUBLIC glut book)
target_compile_features(ray PUBLIC cuda_std_14)

set_target_properties(ray PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)

add_custom_command(TARGET ray POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:glut> $<TARGET_FILE_DIR:ray>
    )

add_executable(ray_noconst ray_noconst.cu)
target_link_libraries(ray_noconst PUBLIC glut book)
target_compile_features(ray_noconst PUBLIC cuda_std_14)

set_target_properties(ray_noconst PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)

add_custom_command(TARGET ray_noconst POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:glut> $<TARGET_FILE_DIR:ray_noconst>
        )