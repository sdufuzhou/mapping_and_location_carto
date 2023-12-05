# 动态库单元测试设置
set(LIB_TEST_NAME mapping_location_test)

set(BOOST_COMPONENTS iostreams)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
# find_package(GLog REQUIRED)
message("#########GLOG_INCLUDE_DIRS=${GLOG_INCLUDE_DIRS}")
message("########GLOG_LIBRARIES=${GLOG_LIBRARIES}")
include_directories(
    ${3rdparty_include}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIR}
    ${ROBOTIC_PROJECT_DIR}/prebuilt/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping_lib/carto_mapping
    
)
set(CMAKE_CXX_FLAGS "-std=c++14 -g -O0 -fprofile-arcs -ftest-coverage")

# add_executable(main
# ${PLUGIN_SRC}
#   main.cpp
# )
add_executable(${LIB_TEST_NAME}
  ${PLUGIN_SRC}
  ${ALL_TEST}
  test_main.cc
)

set(PREBUILT_DIR ${ROBOTIC_PROJECT_DIR}/prebuilt/lib_so)

target_link_libraries(${LIB_TEST_NAME} PUBLIC
    ${GTEST_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES} 
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARY} 
    ${LUA_LIBRARIES}
    gflags
    cairo
    -ljsoncpp
    -lgcov -lz -pthread -lboost_system -lboost_thread -llog4cpp
    absl::algorithm
    absl::base
    absl::debugging
    absl::flat_hash_map
    absl::memory
    absl::meta
    absl::numeric
    absl::str_format_internal
    absl::str_format
    absl::strings
    absl::synchronization
    absl::time
    absl::utility 
    ${PREBUILT_DIR}/libagv_config_lib.so
    ${PREBUILT_DIR}/libcommon_lib.so
)

# gtest_discover_tests(${LIB_TEST_NAME}
#   WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
#   -lglog
#   -lgcov 
#   -lz
#   -pthread
# )
