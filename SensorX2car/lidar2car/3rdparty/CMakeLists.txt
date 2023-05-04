project(3rdparty)
# add_subdirectory(eigen3)
add_subdirectory(ceres)
add_subdirectory(g2o)
add_subdirectory(gflags)
add_subdirectory(glog)
## Define temporary variable
set(TEMP_INCLUDE_DIRS "")
set(TEMP_LINK_DIRS "")

list(APPEND TEMP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND TEMP_LINK_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib)

## Export 3rdparty include/link directories && libraries
set(3RDPATRYS_INCLUDE_DIRS ${TEMP_INCLUDE_DIRS} PARENT_SCOPE)
set(3RDPATRYS_LINK_DIRS ${TEMP_LINK_DIRS} PARENT_SCOPE)
