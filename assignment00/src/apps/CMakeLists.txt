set(OPENGLLIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLEW_LIBRARIES} ${OPENGL_LIBRARY})

if(WIN32)
message(STATUS "Win32: adding glfw32dll and glew32")
set(OPENGLLIBS glfw3dll glew32 ${OPENGLLIBS})
else()
set(OPENGLLIBS glfw ${OPENGLLIBS})
endif()

set(00_srcs  00_build.cpp)                                  # 00_build
add_executable(00_build ${00_srcs})                         # 00_build
target_link_libraries(00_build common ${OPENGLLIBS})        # 00_build
SOURCE_GROUP("" FILES ${00_srcs})                           # 00_build







if(CMAKE_GENERATOR STREQUAL "Xcode")
    set_property(TARGET 00_build      PROPERTY XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD c++11)
    set_property(TARGET 00_build      PROPERTY XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY libc++)
endif(CMAKE_GENERATOR STREQUAL "Xcode")


