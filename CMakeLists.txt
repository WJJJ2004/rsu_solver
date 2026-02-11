cmake_minimum_required(VERSION 3.16)
project(rsu_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(rsu_solver
  src/rsu_solver.cpp
)

target_include_directories(rsu_solver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(rsu_solver
  PUBLIC Eigen3::Eigen
)

# Optional: warnings
if (MSVC)
  target_compile_options(rsu_solver PRIVATE /W4)
else()
  target_compile_options(rsu_solver PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- install (so other projects can find/link it) ----
include(GNUInstallDirs)

install(TARGETS rsu_solver
  EXPORT rsu_solverTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT rsu_solverTargets
  FILE rsu_solverTargets.cmake
  NAMESPACE rsu_solver::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rsu_solver
)

