include_directories(include)

file(GLOB fisheyestitcher_src "src/*.cpp")
file(GLOB fisheyestitcher_head "include/*.h")

#set(OpenCV_DIR "C:/Code-Libraries/OpenCV/OpenCV310/OpenCV310_x64_VS15/install")

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "cannot find the opencv")
endif()

if(WIN32)
  include_directories("${OpenCV_DIR}/include")
endif()
add_definitions(-DCOST_TIME=1)
add_library(FishEyeStitcher SHARED ${fisheyestitcher_src} ${fisheyestitcher_head})

set(dep_libs opencv_core opencv_calib3d opencv_features2d opencv_flann opencv_highgui opencv_imgproc opencv_stitching opencv_xfeatures2d opencv_imgcodecs)
set(version_suffix "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

foreach(d ${dep_libs})
  if(WIN32)
    find_library(${d}_lib_paths NAMES ${d}${version_suffix} PATHS ${OpenCV_LIB_PATH})
    list(APPEND dep_libs_full "${${d}_lib_paths}")
    find_library(${d}_lib_paths_debug NAMES ${d}${version_suffix}d PATHS ${OpenCV_LIB_PATH})
    list(APPEND dep_libs_full_debug "${${d}_lib_paths_debug}")
  endif()
endforeach()

#message(STATUS "${dep_libs}")

if(WIN32)
  target_link_libraries(FishEyeStitcher "$<$<CONFIG:Release>:${dep_libs_full}>" "$<$<CONFIG:Debug>:${dep_libs_full_debug}>")
else()
  target_link_libraries(FishEyeStitcher ${dep_libs})
endif()

if(WIN32)
	install(TARGETS FishEyeStitcher
		RUNTIME DESTINATION ./)
	foreach(d ${dep_libs})
	    find_file(${d}_dll_paths NAMES ${d}${version_suffix}.dll PATHS ${_OpenCV_LIB_PATH})
	    list(APPEND dep_dlls_full "${${d}_dll_paths}")
	endforeach()
	install(FILES ${dep_dlls_full} DESTINATION ./)
endif()










