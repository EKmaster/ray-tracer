cmake_minimum_required(VERSION 3.14)

project(RTInOneWeekend LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_executable(inOneWeekend src/main.cc)
target_include_directories(inOneWeekend PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(inOneWeekend PRIVATE Threads::Threads)

if (MSVC)
  target_compile_options(inOneWeekend PRIVATE $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:/O2>)
  target_compile_options(inOneWeekend PRIVATE $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:/fp:fast>)
  target_compile_options(inOneWeekend PRIVATE $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:/arch:AVX2>)
else ()
  target_compile_options(inOneWeekend PRIVATE $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O3>)
  target_compile_options(inOneWeekend PRIVATE $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-march=native>)
endif ()
