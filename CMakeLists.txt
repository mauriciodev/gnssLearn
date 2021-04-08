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

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(TS_FILES gnssLearn_en_US.ts)

set(SOURCES
rinex3.cpp
gnsstime.cpp
spp.cpp
sp3.cpp
orbitreader.cpp
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/RJNI00BRA_R_20210680000_01D_15S_MO.rnx
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/BRDC00WRD_S_20210680000_01D_MN.rnx
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/gfz21482.sp3
      DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/COD0MGXFIN_20210680000_01D_05M_ORB.SP3
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)


add_library(gnssLearnLib
  ${SOURCES}
  ${TS_FILES}
)
target_link_libraries(gnssLearnLib Qt5::Core)

add_executable(gnssLearn main.cpp)
target_link_libraries(gnssLearn Qt5::Core gnssLearnLib)

add_executable(gnssLearnTests test.cpp)
add_test(NAME gnssLearnTests COMMAND gnssLearnTests)
target_link_libraries(gnssLearnTests PRIVATE Qt5::Test gnssLearnLib)



qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})


