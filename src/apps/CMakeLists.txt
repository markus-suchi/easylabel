#el_accu: depth accumulation app
add_executable(el_accu
  accu_proc.cpp
)
target_link_libraries(el_accu
  easylabel
  v4r_filesystem
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

#el_diff: dept differencing app
add_executable(el_diff
  diff_proc.cpp
)
target_link_libraries(el_diff
  easylabel
  v4r_filesystem
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

#el_pcd2img: app for extracting depth,rgb,label image from pointcloud file
add_executable(el_pcd2img
  pcd2img_proc.cpp
)
target_link_libraries(el_pcd2img
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#el_remove: removing small patches (stray pixels) from pcd files
add_executable(el_remove
  remove_proc.cpp
)
target_link_libraries(el_remove
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

#el_equal: compare 2 cloud files for debuging refactoring: REMOVE
#add_executable(el_equal
#  equal_proc.cpp
#)
#target_link_libraries(el_equal
#  ${PCL_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
