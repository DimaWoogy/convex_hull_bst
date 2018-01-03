project(convex_hull)

cmake_minimum_required(VERSION 3.2)

# CGAL
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
include( CGAL_CreateSingleSourceCGALProgram )
include_directories(BEFORE "../../include")
include_directories(BEFORE "include")

# Boost
find_package(Boost 1.60)
include_directories(${Boost_INCLUDE_DIR})

# Warnings
if (MSVC)
   add_compile_options(/W4)
else()
   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ALGO_DIR algorithms)

include_directories(${ALGO_DIR})
set(ALGO_HEADERS ${ALGO_DIR}/bsthull.h ${ALGO_DIR}/graham.h ${ALGO_DIR}/point.h ${ALGO_DIR}/chan.h)

set(UTILS_DIR utils)
include_directories(${UTILS_DIR})
set(UTILS_HEADERS ${UTILS_DIR}/cyclicset.h)

add_executable(main charts/main.cpp ${ALGO_HEADERS} ${UTILS_HEADERS})
target_link_libraries(main ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

set_property(TARGET main PROPERTY CXX_STANDARD 14)

enable_testing()

add_executable(randomtest test/randomtesting.cpp ${ALGO_HEADERS} ${UTILS_HEADERS})
target_link_libraries(randomtest ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

add_test(Randomtest randomtest)
