# Package-private CPU subset; upstream algorithm sources are unmodified.
add_library(ml_fast_gicp STATIC
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
)
target_include_directories(ml_fast_gicp SYSTEM PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ml_fast_gicp PUBLIC
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
target_compile_options(ml_fast_gicp PRIVATE ${PCL_DEFINITIONS})
