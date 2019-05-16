include_directories(
  ${APP_ROOT_SRC_DIR}/ext/v4r/io/include
)
add_library(v4r_filesystem v4r/io/src/filesystem.cpp)
target_link_libraries(v4r_filesystem 
  ${PCL_LIBRARIES}  
  ${Boost_LIBRARIES}
)
