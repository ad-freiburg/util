file(GLOB_RECURSE util_SRC *.cpp)
list(REMOVE_ITEM util_SRC TestMain.cpp)

if(NOT CMAKE_TESTING_ENABLED)
	list(FILTER util_SRC EXCLUDE REGEX ".*/tests/.*")
endif()

add_library(pb_util ${util_SRC})

target_include_directories(pb_util PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

if (NOT BZIP2_FOUND)
    find_package(BZip2)
endif()

if (BZIP2_FOUND)
	target_compile_definitions(pb_util INTERFACE DPBUTIL_BZIP2_FOUND=${BZIP2_FOUND} )
	target_include_directories(pb_util INTERFACE ${BZIP2_INCLUDE_DIR} )
	target_link_libraries( pb_util ${BZIP2_LIBRARIES} )
endif(BZIP2_FOUND)

if (NOT ZLIB_FOUND)
    find_package(ZLIB)
endif()

if (ZLIB_FOUND)
	target_compile_definitions(pb_util INTERFACE DPBUTIL_ZLIB_FOUND=${ZLIB_FOUND} )
	target_include_directories(pb_util INTERFACE ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries( pb_util ${ZLIB_LIBRARIES} )
endif(ZLIB_FOUND)

if(CMAKE_TESTING_ENABLED)
    add_subdirectory(tests)
endif()
