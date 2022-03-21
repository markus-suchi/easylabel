PROJECT(easylabel)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost COMPONENTS program_options regex REQUIRED )

message("")
message("  C/C++:")
message("    Built as dynamic libs:"  BUILD_SHARED_LIBS THEN YES ELSE NO)
message("    C++ Compiler:"           ${CMAKE_CXX_COMPILER_ARG1})
message("    C++ flags (Release):"    ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE})
message("    C++ flags (Debug):"      ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG})
message("    C Compiler:"             ${CMAKE_C_COMPILER} ${CMAKE_C_COMPILER_ARG1})
message("    C flags (Release):"      ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE})
message("    C flags (Debug):"        ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG})
message("    Linker flags (Release):" ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})
message("    Linker flags (Debug):"   ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG})
message("    OpenCV_LIBRARIES:"       ${OpenCV_LIBRARIES})

set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(APP_ROOT_SRC_DIR ${APP_ROOT_DIR}/src)

# This function will have effect to all the downstream cmakelist files.
include_directories(
    ${APP_ROOT_DIR}
    ${APP_ROOT_SRC_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#easylabel library
add_library(easylabel
  ${APP_ROOT_SRC_DIR}/easylabel/depth_differ.cpp
  ${APP_ROOT_SRC_DIR}/easylabel/depth_accumulator.cpp
  ${APP_ROOT_SRC_DIR}/easylabel/label_helper.cpp
)
target_link_libraries(easylabel
  ${PCL_LIBRARY}
  ${OpenCV_LIBRARIES}
)

#easylabel apps
add_subdirectory(src/apps)

#external dependencies
add_subdirectory(src/ext)


