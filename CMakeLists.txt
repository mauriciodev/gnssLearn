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

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/RJNI00BRA_R_20210680000_01D_15S_MO.rnx
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)
 file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/BRDC00WRD_S_20210680000_01D_MN.rnx
      DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)

add_executable(gnssLearn
  ${SOURCES}
  ${TS_FILES}
)
target_link_libraries(gnssLearn Qt5::Core Qt5::Test)

qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})


