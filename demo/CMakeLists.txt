file(GLOB SOURCES ./*.cpp ./Contrib/fastlz/fastlz.c ./*.h ./Contrib/fastlz/fastlz.h)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(NOT SDL2_ROOT_DIR)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/SDL")
endif()

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
if (WIN32)
    add_executable(demo WIN32 ${SOURCES})
elseif(APPLE)
    add_executable(demo MACOSX_BUNDLE ${SOURCES})
else()
    add_executable(demo ${SOURCES})
endif()

file(COPY ../recastnavigation/RecastDemo/Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ../recastnavigation/RecastDemo/Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ../recastnavigation/RecastDemo/Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

if (WIN32)
    file(COPY "${SDL2_RUNTIME_LIBRARY}" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

add_dependencies(demo DebugUtils Detour DetourCrowd DetourTileCache Recast Plus)
target_link_libraries(demo ${OPENGL_LIBRARIES} SDL2::SDL2main DebugUtils Detour DetourCrowd DetourTileCache Recast Plus)
SET_TARGET_PROPERTIES(demo PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/")
install(TARGETS demo
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)

if (WIN32)
    install(FILES "${SDL2_RUNTIME_LIBRARY}" DESTINATION bin)
endif()