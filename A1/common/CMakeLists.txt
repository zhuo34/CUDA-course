set(book_headers
        book.h
        cpu_bitmap.h
        cpu_anim.h
        gl_helper.h
        gpu_anim.h)
add_library(book INTERFACE ${book_headers})

target_include_directories(book INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})