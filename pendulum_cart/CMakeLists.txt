cmake_minimum_required(VERSION 3.13.4)
project(pendulum_cart)
set (PROJECT_VERSION "1.0"    )
project(${PROJECT_NAME} VERSION ${PROJECT_VERSION})
# location of ma-libs
set(MA_LIBS_ROOT "${CMAKE_SOURCE_DIR}/externals/ma-libs")
set(MA_LIBS_CPP_ROOT "${MA_LIBS_ROOT}/cpp")
list(APPEND CMAKE_MODULE_PATH "${MA_LIBS_ROOT}/cmake_modules")
include( COMPILERCOMMON     )
include( COMPILERCPP        )
include( FindLIBS           )
cmake_policy(SET CMP0072 NEW)
find_package(OpenGL REQUIRED)
set(CMAKE_CXX_STANDARD 17   )

if(WIN32)
    generic_libs_find(glew ON      )
    generic_libs_find(glfw ON      )
    generic_libs_find(glm  ON      )
    generic_libs_find(hdf5   ON       )
    include_directories( ${HDF5_INCLUDE_DIRS}/src )
    include_directories( ${HDF5_INCLUDE_DIRS2}/hdf5 )
else()
    generic_libs_find(glew OFF     )
    generic_libs_find(glfw OFF     )
    generic_libs_find(glm  OFF     )
endif()

include_directories( ${GLEW_INCLUDE_DIRS}           )
include_directories( ${GLEW_INCLUDE_DIRS}/include   )
include_directories( ${GLFW_INCLUDE_DIRS}           )
include_directories( ${GLFW_INCLUDE_DIRS}/include   )
include_directories( ${GLM_INCLUDE_DIRS}            )
include_directories( ${MA_LIBS_CPP_ROOT}                )
include_directories( ${MA_LIBS_CPP_ROOT}/math           )
include_directories( ${MA_LIBS_CPP_ROOT}/libnn/src      )
include_directories( ${MA_LIBS_CPP_ROOT}/libgraphic_engine/src      )
include_directories( ${MA_LIBS_CPP_ROOT}/utils          )

add_definitions( -DLOGGING      )
add_definitions( -DGLEW_STATIC -DGLM_ENABLE_EXPERIMENTAL )

set( SRCS_GUI ./src/gl_app.cpp ./src/gl_window.cpp )
set( SRCS ./src/pendulum_cart.cpp ${MA_LIBS_CPP_ROOT}/utils/log/log.cpp )

link_directories(${LIBS_DIR})

link_directories(${GLFW_LIBRARY_PATH}   )
link_directories(${GLEW_LIBRARY_PATH}   )
link_directories(${GLEW_LIBRARY_PATH2}  )
link_directories( ${MA_LIBS_ROOT}/build/${CMAKE_BUILD_TYPE} )

add_definitions( -DUP_POSITION  )
add_executable( ${PROJECT_NAME} ./src/main1.cpp ${SRCS} ${SRCS_GUI})

if(UNITYBUILD)
    set_property( TARGET ${PROJECT_NAME} PROPERTY UNITY_BUILD ON )
endif()

if (MSVC)
    target_link_libraries( ${PROJECT_NAME} ${OPENGL_gl_LIBRARY} debug cpp_graphic_engined debug cpp_nnd optimized cpp_graphic_engine optimized cpp_nn glfw3 glew)
    macro(COPYDLL)
      if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/../../Release/
          COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_SOURCE_DIR}/externals/ma-libs/build/Release/cpp_alg_lapack.dll
          ${CMAKE_BINARY_DIR}/../../Release/
          COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_SOURCE_DIR}/externals/ma-libs/build/Release/cpp_nn.dll
          ${CMAKE_BINARY_DIR}/../../Release/
          )
      elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/../../Debug/
          COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_SOURCE_DIR}/externals/ma-libs/build/Debug/cpp_alg_lapackd.dll
          ${CMAKE_BINARY_DIR}/../../Debug/
          COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_SOURCE_DIR}/externals/ma-libs/build/Debug/cpp_nnd.dll
          ${CMAKE_BINARY_DIR}/../../Debug/
          )
      endif()
    endmacro()
    cmake_language(CALL COPYDLL)
elseif(APPLE)
    target_link_libraries( ${PROJECT_NAME} ${OPENGL_gl_LIBRARY} debug engined debug cpp_nnd optimized cpp_graphic_engine optimized cpp_nn libglfw.dylib glew "-framework Foundation -framework IOKit -framework Cocoa" )
else()
    target_link_libraries( ${PROJECT_NAME} ${OPENGL_gl_LIBRARY} debug engined debug cpp_nnd optimized cpp_graphic_engine optimized cpp_nn libglfw.so libGLEW.so -ldl -lpthread -lGL -lGLU)
endif()

add_executable( ${PROJECT_NAME}_bin ./src/main2.cpp ${SRCS} )

if(UNITYBUILD)
    set_property( TARGET ${PROJECT_NAME}_bin PROPERTY UNITY_BUILD ON )
endif()

if (MSVC)
    target_link_libraries( ${PROJECT_NAME}_bin debug cpp_nnd optimized cpp_nn )
elseif(APPLE)
    target_link_libraries( ${PROJECT_NAME}_bin debug cpp_nnd optimized cpp_nn )
else()
    target_link_libraries( ${PROJECT_NAME}_bin debug cpp_nnd optimized cpp_nn )
endif()

add_executable( ${PROJECT_NAME}_simul ./src/main3.cpp ${SRCS} )

if(UNITYBUILD)
    set_property( TARGET ${PROJECT_NAME}_simul PROPERTY UNITY_BUILD ON )
endif()

if (MSVC)
    target_link_libraries( ${PROJECT_NAME}_simul debug cpp_nnd optimized cpp_nn )
elseif(APPLE)
    target_link_libraries( ${PROJECT_NAME}_simul debug cpp_nnd optimized cpp_nn )
else()
    target_link_libraries( ${PROJECT_NAME}_simul debug cpp_nnd optimized cpp_nn )
endif()



