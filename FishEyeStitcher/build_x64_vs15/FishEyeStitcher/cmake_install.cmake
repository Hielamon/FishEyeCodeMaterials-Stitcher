# Install script for directory: D:/Academic-Research/My Papers/FishEyeCodeMaterials-Stitcher/FishEyeStitcher/FishEyeStitcher

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "D:/Academic-Research/My Papers/FishEyeCodeMaterials-Stitcher/FishEyeStitcher/build_x64_vs15/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE SHARED_LIBRARY FILES "D:/Academic-Research/My Papers/FishEyeCodeMaterials-Stitcher/FishEyeStitcher/build_x64_vs15/bin/Debug/FishEyeStitcher.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE SHARED_LIBRARY FILES "D:/Academic-Research/My Papers/FishEyeCodeMaterials-Stitcher/FishEyeStitcher/build_x64_vs15/bin/Release/FishEyeStitcher.dll")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE FILE FILES
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_core320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_calib3d320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_features2d320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_flann320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_highgui320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_imgproc320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_stitching320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_xfeatures2d320.dll"
    "D:/Code-Libraries/OpenCV/OpenCV320/OpenCV320_x64_VS15/install/x64/vc14/bin/opencv_imgcodecs320.dll"
    )
endif()

