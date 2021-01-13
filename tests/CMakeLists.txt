cmake_minimum_required(VERSION 3.19)

Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v2.13.3
)
FetchContent_MakeAvailable(Catch2)

add_executable(particle_filter_tests
  ../src/particle_filter.h
  ../src/particle_filter.cpp
  support.cpp
  particle_filter_test.cpp
)

target_compile_features(particle_filter_tests PRIVATE cxx_std_17)

target_link_libraries(particle_filter_tests PRIVATE Catch2::Catch2)