file(GLOB SOURCES ./*.cpp)

if (WIN32)
    add_executable(tool WIN32 ${SOURCES})
elseif(APPLE)
    add_executable(tool MACOSX_BUNDLE ${SOURCES})
else()
    add_executable(tool ${SOURCES})
endif()

add_dependencies(tool DebugUtils Detour DetourCrowd DetourTileCache Recast Plus)
target_link_libraries(tool DebugUtils Detour DetourCrowd DetourTileCache Recast Plus)
SET_TARGET_PROPERTIES(tool PROPERTIES LINK_FLAGS "/SUBSYSTEM:CONSOLE")
SET_TARGET_PROPERTIES(tool PROPERTIES COMPILE_DEFINITIONS "_CONSOLE")
SET_TARGET_PROPERTIES(tool PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/")

SET(TOOL_TESTING ON)
IF(BUILD_TESTING AND  TOOL_TESTING)
	ADD_CUSTOM_COMMAND(
		TARGET tool
		POST_BUILD
		COMMAND tool  "./" 0
		COMMAND tool  "./" 1
		COMMAND tool  "./" 2
		WORKING_DIRECTORY 	${CMAKE_SOURCE_DIR}/bin/
		COMMENT  "gen map object and mesh object"
	)
ENDIF(BUILD_TESTING AND  TOOL_TESTING)
