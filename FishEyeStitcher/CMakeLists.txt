cmake_minimum_required(VERSION 2.6)
if(WIN32)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
else()
  set(CMAKE_INSTALL_PREFIX "/usr/local/FishEyeStitcher" CACHE PATH "Installation Directory")
endif()

macro(use_cxx11)
  if(CMAKE_VERSION VERSION_LESS "3.1")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	  set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
	endif()
  else()
	set(CMAKE_CXX_STANDARD 11)
  endif()
endmacro(use_cxx11)

if(CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configs" FORCE)
endif()

use_cxx11()

project(FishEyeStitcher)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(FishEyeStitcher)
add_subdirectory(FishEyeStitcherTest)
#add_subdirectory(php_fisheye_stitcher)


