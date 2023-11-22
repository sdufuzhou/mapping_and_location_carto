cmake_minimum_required(VERSION 3.10.2)
set(PACKAGE_NAME mapping_and_location)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-ignored-attributes")
macro(remove_warnings)
    add_definitions(-Wno-unused-value -Wno-unknown-pragmas -Wno-sequence-point 
                    -Wno-delete-non-virtual-dtor -Wno-unused-but-set-variable 
                    -Wno-sign-compare -Wno-unused-variable -Wno-return-local-addr 
                    -Wno-unused-function -Wno-deprecated-declarations)
endmacro()
project(${PACKAGE_NAME})

# Determine the architecture
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_CROSSCOMPILING)
    message("Using aarch64 architecture")
    set(3rdparty_include ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/aarch64)
    set(3rdparty_lib ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/aarch64/lib)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    message("Using x86_64 architecture")
    set(3rdparty_include ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/x86_64)
    set(3rdparty_lib ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/x86_64/lib)
endif()

# Find required packages
# set(GLOG_ROOT "/usr/local")
# LIST(APPEND CMAKE_MODULE_PATH /usr/local/stow/absl/lib/cmake/glog)
# LIST(APPEND CMAKE_MODULE_PATH /usr/local/lib/cmake/glog)
# find_package(glog REQUIRED)
message("#########GLOG_INCLUDE_DIRS=${GLOG_INCLUDE_DIRS}")
message("########GLOG_LIBRARIES=${GLOG_LIBRARIES}")

list(APPEND REQUIRED_PACKAGES PCL Ceres Boost Eigen3 Protobuf Lua GFlags GTest)
message("#########GLOG_INCLUDE_DIRS=${GLOG_INCLUDE_DIRS}")
message("########GLOG_LIBRARIES=${GLOG_LIBRARIES}")
find_package(absl REQUIRED )
message("#########GLOG_INCLUDE_DIRS=${GLOG_INCLUDE_DIRS}")
message("########GLOG_LIBRARIES=${GLOG_LIBRARIES}")
foreach(PACKAGE ${REQUIRED_PACKAGES})
    find_package(${PACKAGE} REQUIRED)
    message("Found ${PACKAGE} at ${${PACKAGE}_INCLUDE_DIRS}")
endforeach()



message("#########GLOG_INCLUDE_DIRS=${GLOG_INCLUDE_DIRS}")
message("########GLOG_LIBRARIES=${GLOG_LIBRARIES}")

# Include directories
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

# Source files
file(GLOB_RECURSE PLUGIN_SRC "src/*.cc")
file(GLOB_RECURSE PLUGIN_PUBLIC_H "include/${PACKAGE_NAME}/*")

# Define the library
add_library(${PACKAGE_NAME} SHARED ${PLUGIN_SRC})

# target_include_directories(${PACKAGE_NAME} PUBLIC)

# Link libraries
target_link_libraries(${PACKAGE_NAME} 
# link_libraries(
    ${GLOG_LIBRARIES}
    # glog
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
    ${ROBOTIC_PROJECT_DIR}/prebuilt/lib_so/libagv_config_lib.so
    ${ROBOTIC_PROJECT_DIR}/prebuilt/lib_so/libcommon_lib.so
)

# Set properties
set_target_properties(${PACKAGE_NAME} PROPERTIES PUBLIC_HEADER "${PLUGIN_PUBLIC_H}")

# Install
set(CMAKE_INSTALL_INCLUDEDIR ${ROBOTIC_PROJECT_DIR}/prebuilt/include)
set(CMAKE_INSTALL_LIBDIR ${ROBOTIC_PROJECT_DIR}/prebuilt/${PLATFORM})
install(TARGETS ${PACKAGE_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PACKAGE_NAME}
)

# Compiler flags
set(CMAKE_CXX_FLAGS "-std=c++14 -g -O0 -fprofile-arcs -ftest-coverage")

# Unit tests
add_subdirectory(test)
