# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/runner/workspace/sensor_node/build/_deps/picotool-src"
  "/home/runner/workspace/sensor_node/build/_deps/picotool-build"
  "/home/runner/workspace/sensor_node/build/_deps"
  "/home/runner/workspace/sensor_node/build/picotool/tmp"
  "/home/runner/workspace/sensor_node/build/picotool/src/picotoolBuild-stamp"
  "/home/runner/workspace/sensor_node/build/picotool/src"
  "/home/runner/workspace/sensor_node/build/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/runner/workspace/sensor_node/build/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/runner/workspace/sensor_node/build/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
