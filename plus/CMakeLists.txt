file(GLOB SOURCES ./*.cpp ./*.h)

if(RECASTNAVIGATION_STATIC)
    add_library(Plus STATIC ${SOURCES})
else()
    add_library(Plus SHARED ${SOURCES})
endif()

add_library(RecastNavigation::Plus ALIAS Plus)

set(Detour_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(Plus PUBLIC
    "$<BUILD_INTERFACE:${Detour_INCLUDE_DIR}>"
)

set_target_properties(Plus PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )

install(TARGETS Plus
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES ./*.h)
install(FILES ${INCLUDES} DESTINATION include)