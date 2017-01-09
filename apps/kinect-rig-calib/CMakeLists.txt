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
ADD_EXECUTABLE(kinect-rig-calib
               kinect-rig-calib_main.cpp)

SET(TMP_TARGET_NAME "kinect-rig-calib")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs mrpt-maps mrpt-pbmap mrpt-gui)

DeclareAppForInstall(${TMP_TARGET_NAME})

endif(BUILD_mrpt-pbmap) # This project can't be built without mrpt-pbmap
