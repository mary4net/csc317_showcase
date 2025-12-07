cmake_minimum_required(VERSION 3.20)
project(raytracing)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# PIC
# This sets -fPIC (or equivalent) where needed in a target-scoped way.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Paths
set(ROOT "${CMAKE_CURRENT_LIST_DIR}")

# Use solution or src
option(USE_SOLUTION "Build the reference solution" OFF)
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(SRC_DIR "${SRC_ROOT}")
if (USE_SOLUTION)
  set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/solution")
elseif(EXISTS "${SRC_ROOT}/a3a5")
  # Prefer assignment sources under src/a3a5 when present.
  set(SRC_DIR "${SRC_ROOT}/a3a5")
endif()

# Sources used by the interactive ray-traced viewer
set(RT_SOURCES
  "${SRC_DIR}/PointLight.cpp"
  "${SRC_DIR}/blinn_phong_shading.cpp"
  "${SRC_DIR}/catmull_clark.cpp"
  "${SRC_DIR}/raycolor.cpp"
  "${SRC_DIR}/reflect.cpp"
  "${SRC_DIR}/triangle_area_normal.cpp"
  "${SRC_DIR}/vertex_triangle_adjacency.cpp"
  "${SRC_DIR}/write_obj.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh/mesh_builders.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
)

# Homework library sources
set(HW2FILES
  "${SRC_DIR}/Triangle.cpp"
  "${SRC_DIR}/TriangleSoup.cpp"
  "${SRC_DIR}/first_hit.cpp"
  "${SRC_DIR}/viewing_ray.cpp"
  "${SRC_DIR}/write_ppm.cpp"
)

# Allow caller to provide extra sources (e.g., libigl helpers)
set(EXTRA_SOURCES "")
if (DEFINED LIBIGL_EXTRA_SOURCES)
  list(APPEND EXTRA_SOURCES ${LIBIGL_EXTRA_SOURCES})
endif()

# Executable target
add_executable(${PROJECT_NAME} ${RT_SOURCES} ${EXTRA_SOURCES})

# Include paths (target-scoped). Mark third-party as SYSTEM to reduce warnings.
target_include_directories(${PROJECT_NAME}
  PRIVATE
    "${ROOT}/include"
)
if (EXISTS "${ROOT}/eigen")
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${ROOT}/eigen")
endif()
if (EXISTS "${ROOT}/json")
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${ROOT}/json")
endif()

# ---- Optional external hw2 library support ----
# If you want to link against a prebuilt hw2 in a directory:
#   cmake -DCMAKE_BUILD_TYPE=Debug -DHW2LIB_DIR=../lib/debug/linux/ ..
set(HW2LIB_DIR  "" CACHE PATH   "Directory containing prebuilt hw2 library (optional)")
set(HW2LIB_NAME "hw2" CACHE STRING "Name of the prebuilt hw2 library (optional)")

if (HW2LIB_DIR)
  message(STATUS "Using HW2LIB_DIR: ${HW2LIB_DIR}, HW2LIB_NAME: ${HW2LIB_NAME}")
  # Link an external lib named HW2LIB_NAME from HW2LIB_DIR
  target_link_directories(${PROJECT_NAME} PRIVATE "${HW2LIB_DIR}")
  target_link_libraries(${PROJECT_NAME} PRIVATE ${HW2LIB_NAME})
else()
  message(STATUS "No HW2LIB_DIR provided, building hw2 from source.")
  # Build our own hw2 library from the HW2FILES and link it
  # We're readding the files we removed from above.
  add_library(hw2 STATIC ${HW2FILES})

  # Inherit include paths for third-party headers
  target_include_directories(hw2
    PRIVATE
      "${ROOT}/include"
  )
  if (EXISTS "${ROOT}/eigen")
    target_include_directories(hw2 SYSTEM PRIVATE "${ROOT}/eigen")
  endif()
  if (EXISTS "${ROOT}/json")
    target_include_directories(hw2 SYSTEM PRIVATE "${ROOT}/json")
  endif()

  # Link the hw2 library to the main executable.
  target_link_libraries(${PROJECT_NAME} PRIVATE hw2)
endif()

# Warnings
if (MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
  if (TARGET hw2)
    target_compile_options(hw2 PRIVATE /W4 /permissive-)
  endif()
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
  if (TARGET hw2)
    target_compile_options(hw2 PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

find_package(OpenMP REQUIRED)
if (OpenMP_CXX_FOUND)
    target_link_libraries(raytracing PRIVATE OpenMP::OpenMP_CXX)
endif()

# SDL2 for interactive window/input handling
find_package(SDL2 REQUIRED CONFIG)
target_link_libraries(raytracing PRIVATE SDL2::SDL2)
if (TARGET SDL2::SDL2main)
  target_link_libraries(raytracing PRIVATE SDL2::SDL2main)
endif()
