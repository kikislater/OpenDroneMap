project(odm_texturing)
cmake_minimum_required(VERSION 2.8)

# Set pcl dir to the input spedified with option -DPCL_DIR="path"
set(PCL_DIR "PCL_DIR-NOTFOUND" CACHE "PCL_DIR" "Path to the pcl installation directory")

# Set pcl dir to the input spedified with option -DOPENCV_DIR="path"
set(OPENCV_DIR "OPENCV_DIR-NOTFOUND" CACHE "OPENCV_DIR" "Path to the OpenCV installation directory")

# Add compiler options.
add_definitions(-Wall -Wextra)

# Find pcl at the location specified by PCL_DIR
find_package(PCL 1.7 HINTS "${PCL_DIR}/share/pcl-1.7" REQUIRED)

# Find OpenCV.
find_package(OpenCV 2.4 PATHS "${OPENCV_DIR}/share/OpenCV")
if(OpenCV_FOUND)
    set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_stitching)
    add_definitions(-DHASBLENDING)
    message("Using OpenCV 2.4, blending enabled")
else()
    # Failed to find OpenCV 2.4, looking for 2.3 instead
    find_package(OpenCV 2.3 REQUIRED PATHS "${OPENCV_DIR}/share/OpenCV")
    if(OpenCV_FOUND)
        set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui)
        message("OpenCV 2.3 detected, texture blending disabled")
    else()
        message(FATAL_ERROR "Unable to find a supported version of OpenCV. Has to be  2.4.x (recommended) or 2.3.x")
    endif()
endif()

# Add the PCL, Eigen and OpenCV include dirs.
# Necessary since the PCL_INCLUDE_DIR variable set by find_package is broken.)
include_directories(${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR})
include_directories(${EIGEN_ROOT})
include_directories(${OpenCV_INCLUDE_DIRS})

# Add source directory
aux_source_directory("./src" SRC_LIST)

# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(odm_texturing ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${OpenCV_LIBS})
