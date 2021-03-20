cmake_minimum_required(VERSION 3.5)

project(gnssLearn LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core LinguistTools REQUIRED)

find_package(Qt5Test REQUIRED)
enable_testing(true)
set(CMAKE_AUTOMOC ON)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(TS_FILES gnssLearn_en_US.ts)

set(SOURCES
main.cpp
rinex3.cpp
gnsstime.cpp
sps.cpp
)

add_executable(gnssLearn
  ${SOURCES}
  ${TS_FILES}
)
target_link_libraries(gnssLearn Qt5::Core Qt5::Test)

qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})


