file(GLOB_RECURSE util_SRC *.cpp)
list(REMOVE_ITEM util_SRC TestMain.cpp)
add_library(pb_util ${util_SRC})

target_include_directories(pb_util PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

if (!BZIP2_FOUND)
    find_package(BZip2)
    if (BZIP2_FOUND)
		add_definitions( -DBZIP2_FOUND=${BZIP2_FOUND} )
	endif()
endif()

if (BZIP2_FOUND)
	target_include_directories(pb_util INTERFACE ${BZIP2_INCLUDE_DIR} )
	target_link_libraries( pb_util ${BZIP2_LIBRARIES} )
endif(BZIP2_FOUND)

if (!ZLIB_FOUND)
    find_package(ZLIB)
    if (ZLIB_FOUND)
		add_definitions( -DZLIB_FOUND=${ZLIB_FOUND} )
	endif()
endif()

if (ZLIB_FOUND)
	target_include_directories(pb_util INTERFACE ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries( pb_util ${ZLIB_LIBRARIES} )
endif(ZLIB_FOUND)

add_subdirectory(tests)
