# Define the pb_util library
file(GLOB util_SRC *.cpp 3rdparty/*.cpp log/*.cpp)
if(NOT TARGET pb_util)
    add_library(pb_util SHARED ${util_SRC})
    set_target_properties(pb_util PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(pb_util PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
    )
endif()

# Conditionally link util library only for non-Windows platforms
if(NOT WIN32 AND NOT MINGW)
    if(TARGET util)
        target_link_libraries(pb_util PRIVATE util)
    endif()
endif()

# Define the pb_util_geo library
file(GLOB_RECURSE util_geoSRC geo/*.cpp)
if(NOT TARGET pb_util_geo)
    add_library(pb_util_geo SHARED ${util_geoSRC})
    set_target_properties(pb_util_geo PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(pb_util_geo PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
    )
endif()

# Define the pb_util_json library
file(GLOB_RECURSE util_jsonSRC json/*.cpp)
if(NOT TARGET pb_util_json)
    add_library(pb_util_json SHARED ${util_jsonSRC})
    set_target_properties(pb_util_json PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_link_libraries(pb_util_json PUBLIC loom_includes)
endif()

# Define the pb_util_graph library
file(GLOB_RECURSE util_graphSRC graph/*.cpp)
if(NOT TARGET pb_util_graph)
    add_library(pb_util_graph SHARED ${util_graphSRC})
    set_target_properties(pb_util_graph PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(pb_util_graph PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>  # Include the parent directory
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>      # Include the current directory
    )
endif()

# Define the pb_util_http library
file(GLOB_RECURSE util_httpSRC http/*.cpp)
if(NOT TARGET pb_util_http)
    add_library(pb_util_http SHARED ${util_httpSRC})
    set_target_properties(pb_util_http PROPERTIES POSITION_INDEPENDENT_CODE ON)
    # Only ONE target_link_libraries call, keyword signature
    target_link_libraries(pb_util_http PUBLIC loom_includes ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES})
endif()

# Define the pb_util_xml library
file(GLOB_RECURSE util_xmlSRC xml/*.cpp)
if(NOT TARGET pb_util_xml)
    add_library(pb_util_xml SHARED ${util_xmlSRC})
    set_target_properties(pb_util_xml PROPERTIES POSITION_INDEPENDENT_CODE ON)
    # Only ONE target_link_libraries call, keyword signature
    target_link_libraries(pb_util_xml PUBLIC loom_includes ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES})
endif()

# Link BZip2 if found
if (NOT BZIP2_FOUND)
    find_package(BZip2)
endif()

if (BZIP2_FOUND)
    target_compile_definitions(pb_util INTERFACE DPBUTIL_BZIP2_FOUND=${BZIP2_FOUND} )
    target_include_directories(pb_util INTERFACE ${BZIP2_INCLUDE_DIR} )
    target_link_libraries(pb_util PUBLIC ${BZIP2_LIBRARIES})

    target_compile_definitions(pb_util_http INTERFACE DPBUTIL_BZIP2_FOUND=${BZIP2_FOUND} )
    target_include_directories(pb_util_http INTERFACE ${BZIP2_INCLUDE_DIR} )
    target_link_libraries(pb_util_http PUBLIC ${BZIP2_LIBRARIES})

    target_compile_definitions(pb_util_xml INTERFACE DPBUTIL_BZIP2_FOUND=${BZIP2_FOUND} )
    target_include_directories(pb_util_xml INTERFACE ${BZIP2_INCLUDE_DIR} )
    target_link_libraries(pb_util_xml PUBLIC ${BZIP2_LIBRARIES})
endif(BZIP2_FOUND)

if (NOT ZLIB_FOUND)
    find_package(ZLIB)
endif()

if (ZLIB_FOUND)
    target_compile_definitions(pb_util INTERFACE DPBUTIL_ZLIB_FOUND=${ZLIB_FOUND} )
    target_include_directories(pb_util INTERFACE ${ZLIB_INCLUDE_DIRS} )
    target_link_libraries(pb_util PUBLIC ${ZLIB_LIBRARIES})

    target_compile_definitions(pb_util_http INTERFACE DPBUTIL_ZLIB_FOUND=${ZLIB_FOUND} )
    target_include_directories(pb_util_http INTERFACE ${ZLIB_INCLUDE_DIRS} )
    target_link_libraries(pb_util_http PUBLIC ${ZLIB_LIBRARIES})

    target_compile_definitions(pb_util_http INTERFACE DPBUTIL_ZLIB_FOUND=${ZLIB_FOUND} )
    target_include_directories(pb_util_http INTERFACE ${ZLIB_INCLUDE_DIRS} )
    target_link_libraries(pb_util_http PUBLIC ${ZLIB_LIBRARIES})
endif(ZLIB_FOUND)

if(CMAKE_TESTING_ENABLED)
    add_subdirectory(tests)
endif()

# after pb_util is created:
if(TARGET pb_util AND NOT TARGET util)
    add_library(util ALIAS pb_util)
endif()
