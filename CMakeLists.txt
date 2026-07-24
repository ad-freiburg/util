# standalone builds
cmake_minimum_required(VERSION 3.21)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	project(pb_util CXX)
	set(PB_UTIL_STANDALONE ON)

	# we have several projects using this lib which are still on C++11
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_BUILD_TYPE Release)

	# be very strict here
	add_compile_options(-O3 -fno-signed-zeros -fno-trapping-math -Wno-format-extra-args -Wformat-nonliteral -Wformat-security -Wno-implicit-fallthrough -Wnarrowing -Wall -Wextra -Werror -pedantic -pedantic-errors -Woverloaded-virtual -Wnon-virtual-dtor -Wformat=2)
	enable_testing()
endif()

file(GLOB util_SRC *.cpp 3rdparty/*.cpp log/*.cpp)
add_library(pb_util ${util_SRC})
target_include_directories(pb_util PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

file(GLOB_RECURSE util_geoSRC geo/*.cpp)
add_library(pb_util_geo ${util_geoSRC})
target_include_directories(pb_util_geo PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

file(GLOB_RECURSE util_jsonSRC json/*.cpp)
add_library(pb_util_json ${util_jsonSRC})
target_include_directories(pb_util_json PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

file(GLOB_RECURSE util_xmlSRC xml/*.cpp)
add_library(pb_util_xml ${util_xmlSRC})
target_include_directories(pb_util_xml PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

file(GLOB_RECURSE util_graphSRC graph/*.cpp)
add_library(pb_util_graph ${util_graphSRC})
target_include_directories(pb_util_graph PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

file(GLOB_RECURSE util_httpSRC http/*.cpp)
add_library(pb_util_http ${util_httpSRC})
target_include_directories(pb_util_http PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

if (NOT BZIP2_FOUND)
    find_package(BZip2)
endif()

if (BZIP2_FOUND)
	target_compile_definitions(pb_util PUBLIC PBUTIL_BZLIB_FOUND=1 )
	target_include_directories(pb_util PUBLIC ${BZIP2_INCLUDE_DIR} )
	target_link_libraries( pb_util ${BZIP2_LIBRARIES} )

	target_compile_definitions(pb_util_http PUBLIC PBUTIL_BZLIB_FOUND=1 )
	target_include_directories(pb_util_http PUBLIC ${BZIP2_INCLUDE_DIR} )
	target_link_libraries(pb_util_http ${BZIP2_LIBRARIES})

	target_compile_definitions(pb_util_xml PUBLIC PBUTIL_BZLIB_FOUND=1 )
	target_include_directories(pb_util_xml PUBLIC ${BZIP2_INCLUDE_DIR} )
	target_link_libraries(pb_util_xml ${BZIP2_LIBRARIES})
endif(BZIP2_FOUND)

if (NOT ZLIB_FOUND)
    find_package(ZLIB)
endif()

if (ZLIB_FOUND)
	target_compile_definitions(pb_util PUBLIC PBUTIL_ZLIB_FOUND=1 )
	target_include_directories(pb_util PUBLIC ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries(pb_util ${ZLIB_LIBRARIES})

	target_compile_definitions(pb_util_http PUBLIC PBUTIL_ZLIB_FOUND=1 )
	target_include_directories(pb_util_http PUBLIC ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries(pb_util_http ${ZLIB_LIBRARIES})

	target_compile_definitions(pb_util_http PUBLIC PBUTIL_ZLIB_FOUND=1 )
	target_include_directories(pb_util_http PUBLIC ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries(pb_util_http ${ZLIB_LIBRARIES})

	target_compile_definitions(pb_util_xml PUBLIC PBUTIL_ZLIB_FOUND=1 )
	target_include_directories(pb_util_xml PUBLIC ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries(pb_util_xml ${ZLIB_LIBRARIES})
endif(ZLIB_FOUND)

if(CMAKE_TESTING_ENABLED)
    add_subdirectory(tests)
endif()
