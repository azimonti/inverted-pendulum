cmake_minimum_required(VERSION 3.13.4)
project(inverted-pendulum)
set (PROJECT_VERSION "1.0"    )
project(${PROJECT_NAME} VERSION ${PROJECT_VERSION})

add_subdirectory( pendulum      )
add_subdirectory( pendulum_cart )
