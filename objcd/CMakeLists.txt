add_executable(objcd
        objcd.cu
        tri_contact.cu tri_contact.h
        objviewer.cu objviewer.h
        timer.cpp timer.h
        )
target_compile_features(objcd PUBLIC cuda_std_17)

set_target_properties(objcd PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)

#add_custom_command(TARGET objcd POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:glut> $<TARGET_FILE_DIR:objcd>
#    )
