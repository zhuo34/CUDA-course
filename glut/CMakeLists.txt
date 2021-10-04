add_library(glut SHARED IMPORTED GLOBAL)
set_target_properties(glut PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/glut64.dll)
set_target_properties(glut PROPERTIES IMPORTED_IMPLIB ${CMAKE_CURRENT_SOURCE_DIR}/glut64.lib)
#add_library(glut STATIC IMPORTED GLOBAL)
#set_target_properties(glut PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/glut64.lib)

target_include_directories(glut INTERFACE .)