add_executable(shell-app main.cpp Ackermann.cpp Robot.cpp PidController.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
