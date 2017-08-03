INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

if(BUILD_mrpt-pbmap) # This project can't be built without mrpt-pbmap

#-----------------------------------------------------------------
# CMake file for the MRPT application:  icp-slam
#
#  Run with "cmake ." at the root directory
#
#  January 2017, Eduardo Fernández-Moral <efernandezmoral@gmail.com>
#-----------------------------------------------------------------
PROJECT(kinect-rig-calib)


# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME}
                kinect-rig-calib_main.cpp
                kinect-rig-calib.h
                kinect-rig-calib.cpp
                kinect-rig-calib_misc.h
                extrinsic_calib.h
                feat_corresp.h
                feat_corresp.cpp
                extrinsic_calib_lines.h
                extrinsic_calib_lines.cpp
                line_corresp.h
                line_corresp.cpp
                extrinsic_calib_planes.h
                extrinsic_calib_planes.cpp
                plane_corresp.h
                plane_corresp.cpp
                DownsampleRGBD.h
)


# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-obs mrpt-vision mrpt-maps mrpt-pbmap mrpt-gui)

DeclareAppForInstall(${PROJECT_NAME})

endif(BUILD_mrpt-pbmap) # This project can't be built without mrpt-pbmap
