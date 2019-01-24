CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(recast_plus)

ENABLE_TESTING()
SET(BUILD_TESTING OFF)


IF(NOT MSVC_VERSION)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions -Wno-unused-local-typedefs -ffunction-sections -fdata-sections")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs -Wreorder -ffunction-sections -fdata-sections")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")

ELSE(NOT MSVC_VERSION)
#MSVC编译选项
  SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3 /MP /wd4496 /wd4267 /wd4244 /Zi /std:c++latest")
  SET(CMAKE_CXX_FLAGS_DEBUG "-DDEBUG /MTd /Od")
  SET(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG /MT /DEBUG:FASTLINK")
  SET(CMAKE_C_FLAGS_DEBUG "-DDEBUG /MTd")
  SET(CMAKE_C_FLAGS_RELEASE "-DNDEBUG /MT /DEBUG:FASTLINK ")
#预定义宏
  ADD_DEFINiTIONS(-D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -D_WINSOCK_DEPRECATED_NO_WARNINGS) 
ENDIF(NOT MSVC_VERSION)

SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}/")
SET(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE}/")

IF(NOT BOOST_ROOT)
	MESSAGE(FATAL_ERROR, "you must set boost_root")
ELSE(NOT BOOST_ROOT)
	MESSAGE(STATUS "${BOOST_ROOT}")
ENDIF()
FIND_PACKAGE(boost)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/recastnavigation/DebugUtils/Include)
include_directories(${CMAKE_SOURCE_DIR}/recastnavigation/Detour/Include)
include_directories(${CMAKE_SOURCE_DIR}/recastnavigation/DetourCrowd/Include)
include_directories(${CMAKE_SOURCE_DIR}/recastnavigation/DetourTileCache/Include)
include_directories(${CMAKE_SOURCE_DIR}/recastnavigation/Recast/Include)
include_directories(${CMAKE_SOURCE_DIR}/recastnavigation/Include)
include_directories(${CMAKE_SOURCE_DIR}/plus)

ADD_SUBDIRECTORY(recastnavigation)
ADD_SUBDIRECTORY(plus)
ADD_SUBDIRECTORY(demo)
ADD_SUBDIRECTORY(tool)
