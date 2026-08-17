find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rerun_cpp: web-based visualization backend for ui::PangolinWindow (see src/ui/pangolin_window.cc),
# replacing the old Pangolin/OpenGL native window. Pin this to whatever version the frontend's
# @rerun-io/web-viewer-react package uses -- a C++ SDK and JS viewer of different versions cannot
# talk to each other.
set(LIGHTNING_RERUN_VERSION "0.36.0" CACHE STRING "rerun_cpp_sdk release tag, must match @rerun-io/web-viewer-react version")

include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/download/${LIGHTNING_RERUN_VERSION}/rerun_cpp_sdk.zip)

# rerun_sdk compiles ~hundreds of generated C++ files. CMAKE_CXX_FLAGS(_RELEASE) above keep -g -ggdb
# even in Release builds, which makes each of those translation units expensive enough to OOM on
# memory-constrained machines when several compile in parallel. Strip debug info for this one
# third-party target only (nobody needs to step through generated rerun_sdk code) and restore the
# project's normal flags immediately after, so the rest of the codebase is unaffected.
set(_lightning_saved_cxx_flags "${CMAKE_CXX_FLAGS}")
set(_lightning_saved_cxx_flags_release "${CMAKE_CXX_FLAGS_RELEASE}")
foreach (_flag "-ggdb" "-g")
    string(REPLACE "${_flag}" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "${_flag}" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
endforeach ()

FetchContent_MakeAvailable(rerun_sdk)

set(CMAKE_CXX_FLAGS "${_lightning_saved_cxx_flags}")
set(CMAKE_CXX_FLAGS_RELEASE "${_lightning_saved_cxx_flags_release}")

# OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

if (BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
else ()
    add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif ()

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDR_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp
)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(third_party_libs
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        glog gflags
        ${yaml-cpp_LIBRARIES}
        ${pcl_conversions_LIBRARIES}
        tbb
        ${rosbag2_cpp_LIBRARIES}
        rerun_sdk
)

