cmake_minimum_required(VERSION 3.16)
project(chaos_pendulum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Homebrew installs into /opt/homebrew on Apple Silicon.
if(EXISTS "/opt/homebrew")
  list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video highgui)

add_executable(pendulum-track
  src/main.cpp
  src/video_source.cpp
  src/config.cpp
  src/autocal.cpp
  src/tracker.cpp
  src/state_machine.cpp
  src/valleys.cpp
  src/logger.cpp
)

add_executable(pendulum-calibrate
  src/calibrate.cpp
  src/video_source.cpp
  src/config.cpp
  src/valleys.cpp
)

add_executable(pendulum-photo
  src/photo_main.cpp
  src/autocal.cpp
  src/video_source.cpp
  src/config.cpp
  src/valleys.cpp
)

foreach(target pendulum-track pendulum-calibrate pendulum-photo)
  target_include_directories(${target} PRIVATE src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${target} PRIVATE ${OpenCV_LIBS})
endforeach()
