# Shared Autoware NDT core; upstream algorithm sources are unmodified.
add_library(ml_autoware_ndt STATIC
  src/ndt_omp/estimate_covariance.cpp
  src/ndt_omp/multigrid_ndt_omp.cpp
  src/ndt_omp/multi_voxel_grid_covariance_omp.cpp
)
target_include_directories(ml_autoware_ndt SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ml_autoware_ndt PUBLIC
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
target_compile_options(ml_autoware_ndt PRIVATE ${PCL_DEFINITIONS})

# Match upstream's x86 SIMD settings, including all consumers of Eigen types.
# Do not introduce -march=native independently in this library or its callers.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_compile_options(ml_autoware_ndt PUBLIC
    -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2
  )
endif()
