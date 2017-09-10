INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  icp-slam
#
#  Run with "cmake ." at the root directory
#
#  August 2017, Eduardo Fernández-Moral <efernandezmoral@gmail.com>
#-----------------------------------------------------------------
PROJECT(camera-rot-tracker)


# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME}
                camera-rot-tracker_main.cpp
                camera-rot-tracker.cpp      camera-rot-tracker.h
                DifOdometry_Datasets_RGBD.cpp   DifOdometry_Datasets_RGBD.h
                pinhole_model_warp.h
                ../kinect-rig-calib/extrinsic_calib.h
                ../kinect-rig-calib/feat_corresp.cpp            ../kinect-rig-calib/feat_corresp.h
                ../kinect-rig-calib/extrinsic_calib_lines.cpp   ../kinect-rig-calib/extrinsic_calib_lines.h
)


# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-obs mrpt-vision mrpt-maps mrpt-pbmap mrpt-gui)

DeclareAppForInstall(${PROJECT_NAME})

