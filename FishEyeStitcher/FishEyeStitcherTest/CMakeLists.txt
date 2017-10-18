
add_executable(FishEyeStitcherTest FishEyeStitcherTest.cpp)
#message(STATUS "${CMAKE_BINARY_DIR}")
set(FishEyeStitcher_DIR "${CMAKE_BINARY_DIR}/bin")
if(WIN32)
  target_link_libraries(FishEyeStitcherTest "${FishEyeStitcher_DIR}/$<$<CONFIG:Release>:Release>$<$<CONFIG:Debug>:Debug>/FishEyeStitcher.lib")
else()
  target_link_libraries(FishEyeStitcherTest "${FishEyeStitcher_DIR}/libFishEyeStitcher.so")
endif()

if(WIN32)
install(TARGETS FishEyeStitcherTest
	 RUNTIME DESTINATION ./)
endif()