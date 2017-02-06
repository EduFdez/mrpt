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
                #calib_from_planes3D.h
                #calib_from_planes3D.cpp
                extrinsic_calib_pair_xtion.h
                extrinsic_calib_pair_xtion.cpp
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
