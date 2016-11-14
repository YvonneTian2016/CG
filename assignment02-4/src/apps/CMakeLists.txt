set(OPENGLLIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLEW_LIBRARIES} ${OPENGL_LIBRARY})

if(WIN32)
message(STATUS "Win32: adding glfw32dll and glew32")
set(OPENGLLIBS glfw3dll glew32 ${OPENGLLIBS})
else()
set(OPENGLLIBS glfw ${OPENGLLIBS})
endif()



set(02_srcs  02_model.cpp)                                  # 02_model
add_executable(02_model ${02_srcs})                         # 02_model
target_link_libraries(02_model common ${OPENGLLIBS})        # 02_model
SOURCE_GROUP("" FILES ${02_srcs})                           # 02_model






if(CMAKE_GENERATOR STREQUAL "Xcode")
    set_property(TARGET   02_model    PROPERTY XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD c++11)
    set_property(TARGET   02_model    PROPERTY XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY libc++)
endif(CMAKE_GENERATOR STREQUAL "Xcode")


