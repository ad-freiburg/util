file(GLOB_RECURSE utilTest_SRC *.cpp)

add_library(pb_utilTest ${utilTest_SRC})

target_include_directories(pb_utilTest PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
)

target_compile_definitions(pb_utilTest PRIVATE
	TEST_DATASETS="${CMAKE_CURRENT_SOURCE_DIR}/datasets"
)

# avoid recompiling Geo.h for each test
if (NOT CMAKE_DISABLE_PRECOMPILE_HEADERS)
	target_precompile_headers(pb_utilTest PRIVATE <util/geo/Geo.h>)
endif()

add_executable(utilTest TestMain.cpp)
target_link_libraries(utilTest pb_utilTest pb_util pb_util_geo pb_util_graph pb_util_json)

# Register with ctest when building standalone; parent projects keep managing
# their own test harness.
if(PB_UTIL_STANDALONE)
	add_test(NAME utilTest COMMAND utilTest)
endif()
