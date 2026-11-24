cmake_minimum_required(VERSION 3.24)
project(skybag LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(SKYBAG_BUILD_TESTS "Build unit tests" ON)
# e.g. -DSKYBAG_SANITIZERS=address,undefined
set(SKYBAG_SANITIZERS "" CACHE STRING "Comma-separated -fsanitize= list")

if(SKYBAG_SANITIZERS)
  add_compile_options(-fsanitize=${SKYBAG_SANITIZERS} -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=${SKYBAG_SANITIZERS})
endif()

# No fused multiply-add: the same build gives the same numbers on every CPU.
add_compile_options(-ffp-contract=off)
set(SKYBAG_WARNINGS -Wall -Wextra -Wpedantic -Werror)

# terra: the globe engine (geodesy, tile pyramid, LOD, software rasterizer).
add_library(terra_core src/terra/ellipsoid.cpp src/terra/tiles.cpp src/terra/terrain.cpp src/terra/camera.cpp
            src/terra/select.cpp src/terra/mesh.cpp src/terra/render.cpp)
target_include_directories(terra_core PUBLIC include)
target_compile_options(terra_core PRIVATE ${SKYBAG_WARNINGS})

# skybag: the flight bag.
add_library(skybag_core src/skybag/units.cpp src/skybag/geo.cpp src/skybag/navdb.cpp src/skybag/aircraft.cpp
            src/skybag/wind.cpp src/skybag/perf.cpp src/skybag/planner.cpp src/skybag/wb.cpp src/skybag/notam.cpp
            src/skybag/terrain.cpp src/skybag/track.cpp src/skybag/json.cpp src/skybag/http.cpp src/skybag/api.cpp)
target_include_directories(skybag_core PUBLIC include)
target_link_libraries(skybag_core PUBLIC terra_core)
target_compile_options(skybag_core PRIVATE ${SKYBAG_WARNINGS})
target_compile_definitions(skybag_core PUBLIC SKYBAG_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(skybag tools/skybag.cpp)
target_link_libraries(skybag PRIVATE skybag_core)
target_compile_options(skybag PRIVATE ${SKYBAG_WARNINGS})

add_executable(terra tools/terra.cpp)
target_link_libraries(terra PRIVATE terra_core)
target_compile_options(terra PRIVATE ${SKYBAG_WARNINGS})

if(SKYBAG_BUILD_TESTS)
  include(FetchContent)
  # Vendored tarball when present (offline machines), else GitHub; same hash.
  set(_gtest_tarball ${CMAKE_CURRENT_SOURCE_DIR}/third_party/googletest-1.15.2.tar.gz)
  if(EXISTS ${_gtest_tarball})
    set(_gtest_url ${_gtest_tarball})
  else()
    set(_gtest_url https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
  endif()
  FetchContent_Declare(googletest
    URL ${_gtest_url}
    URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)
  add_executable(terra_tests tests/geo_test.cpp tests/tiles_test.cpp tests/raster_test.cpp)
  target_link_libraries(terra_tests PRIVATE terra_core GTest::gtest_main)
  target_compile_options(terra_tests PRIVATE ${SKYBAG_WARNINGS})
  gtest_discover_tests(terra_tests DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 60)

  add_executable(skybag_tests tests/skybag_geo_test.cpp tests/navdb_test.cpp tests/perf_test.cpp
                 tests/planner_test.cpp tests/wb_test.cpp tests/notam_test.cpp tests/http_test.cpp)
  target_link_libraries(skybag_tests PRIVATE skybag_core GTest::gtest_main)
  target_compile_options(skybag_tests PRIVATE ${SKYBAG_WARNINGS})
  gtest_discover_tests(skybag_tests DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 120)

  add_test(NAME globe_checks COMMAND terra --check)
  add_test(NAME efb_checks COMMAND skybag --check)
endif()
