cmake_minimum_required(VERSION 3.16)
project(rovtio_rosfree CXX)

# The upstream CMakeLists is catkin-only and additionally references variables that
# modern CMake's FindGLUT/FindGLEW/FindOpenGL no longer set (${GLUT_LIBRARY},
# ${GLEW_LIBRARY}, ${OpenGL_INCLUDE_DIRS}), so it links nothing. This file replaces
# it entirely; the upstream one is never configured.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(OpenGL_GL_PREFERENCE GLVND)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Staged source tree produced by scripts/stage_sources.sh
#   upstream ROVIO  <-  ROVTIO contribution  <-  our fixes
set(ROVTIO_STAGED "" CACHE PATH "staged source tree")
if(NOT ROVTIO_STAGED)
  message(FATAL_ERROR "set -DROVTIO_STAGED=<dir> (run scripts/stage_sources.sh first)")
endif()

set(ROVIO_NMAXFEATURE 25 CACHE STRING "features tracked")
set(ROVIO_NCAM        2  CACHE STRING "cameras")
set(ROVIO_NLEVELS     4  CACHE STRING "pyramid levels")
set(ROVIO_PATCHSIZE   6  CACHE STRING "patch edge length")
set(ROVIO_NPOSE       0  CACHE STRING "extra poses")
option(MAKE_SCENE "build the OpenGL scene viewer" ON)
option(ROVTIO_NATIVE_ARCH "use -mcpu=native (not portable)" OFF)

find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d)
find_package(Boost REQUIRED)          # header-only: property_tree
find_package(Threads REQUIRED)

find_package(yaml-cpp QUIET CONFIG)
if(TARGET yaml-cpp::yaml-cpp)
  set(ROVTIO_YAMLCPP yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(ROVTIO_YAMLCPP yaml-cpp)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAMLCPP REQUIRED IMPORTED_TARGET yaml-cpp>=0.5)
  set(ROVTIO_YAMLCPP PkgConfig::YAMLCPP)
endif()

# highgui is optional: only the tracker overlay windows need it
find_package(OpenCV QUIET COMPONENTS highgui)
if(OpenCV_highgui_FOUND OR TARGET opencv_highgui)
  set(ROVTIO_HAS_HIGHGUI ON)
endif()

add_library(rovtio_deps INTERFACE)
target_include_directories(rovtio_deps INTERFACE ${ROVTIO_STAGED}/include)
target_link_libraries(rovtio_deps INTERFACE Eigen3::Eigen Boost::headers)

add_library(rovtio_core STATIC
  ${ROVTIO_STAGED}/src/Camera.cpp
  ${ROVTIO_STAGED}/src/FeatureCoordinates.cpp
  ${ROVTIO_STAGED}/src/FeatureDistance.cpp)
if(MAKE_SCENE)
  target_sources(rovtio_core PRIVATE ${ROVTIO_STAGED}/src/Scene.cpp)
endif()
target_compile_definitions(rovtio_core PUBLIC
  ROVIO_NMAXFEATURE=${ROVIO_NMAXFEATURE}
  ROVIO_NCAM=${ROVIO_NCAM}
  ROVIO_NLEVELS=${ROVIO_NLEVELS}
  ROVIO_PATCHSIZE=${ROVIO_PATCHSIZE}
  ROVIO_NPOSE=${ROVIO_NPOSE})
target_link_libraries(rovtio_core PUBLIC rovtio_deps ${OpenCV_LIBS} ${ROVTIO_YAMLCPP})

if(MAKE_SCENE)
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
  find_package(GLEW REQUIRED)
  target_compile_definitions(rovtio_core PUBLIC MAKE_SCENE=1)
  target_link_libraries(rovtio_core PUBLIC OpenGL::GL GLUT::GLUT GLEW::GLEW)
endif()

add_executable(rovtio_player src/main_player.cpp src/DatasetPlayer.cpp)
target_link_libraries(rovtio_player PRIVATE rovtio_core Threads::Threads)
if(ROVTIO_HAS_HIGHGUI)
  target_compile_definitions(rovtio_player PRIVATE ROVTIO_WITH_HIGHGUI=1)
endif()

# -march=native on aarch64 resolves to whatever the build host has (under Docker on
# Apple silicon that includes i8mm/bf16), which SIGILLs on other arm64 parts.
# Default to a baseline every relevant target shares.
if(ROVTIO_NATIVE_ARCH)
  add_compile_options(-mcpu=native)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  add_compile_options(-march=armv8.2-a+fp16+dotprod)
endif()

install(TARGETS rovtio_player DESTINATION bin)
install(DIRECTORY ${ROVTIO_STAGED}/shaders DESTINATION .)
