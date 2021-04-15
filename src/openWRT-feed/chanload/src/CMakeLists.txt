# This file is part of Lanradio.
#
# Lanradio is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Lanradio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Lanradio.  If not, see <http://www.gnu.org/licenses/>.
#
# Authors:
# Florian Klingler <klingler@ccs-labs.org>

cmake_minimum_required(VERSION 2.8)

project(chanload C CXX)

SET(PROJECT_VERSION_MAJOR "1")
SET(PROJECT_VERSION_MINOR "0")
SET(CMAKE_CXX_FLAGS "-g -O0 -std=c++11 -Wall")
# SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}, ${CMAKE_SOURCE_DIR}/CMake/Modules")
# SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake/Modules)
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake/Modules)
message("---------------------------!!!!mario!!!!!-----------------------------")
message(${CMAKE_MODULE_PATH})

find_package(Boost COMPONENTS date_time system thread)

include_directories(
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
)
SET(LIBS chanload pthread boost_system boost_thread)
# SET(LIBS chanload pthread boost_system boost_thread utility)

set(chanload_SRCS
   chanload.cc
   LoggingUtility.cpp
)

find_package(NL)
SET(LIBS ${LIBS} ${NL_LIBRARIES})
include_directories(
    ${NL_INCLUDE_DIRS}
)
# message("found nl libraries")
# message(${NL_INCLUDE_DIRS})

# find_package (LIBNL-TINY REQUIRED)
# SET(LIBS ${LIBS} ${LIBNL-TINY_LIBRARIES})
# include_directories(
#     ${LIBNL-TINY_INCLUDE_DIRS}
# )
# message("found nl tiny libraries")
# message(${LIBNL-TINY_INCLUDE_DIRS})

add_executable (chanload ${chanload_SRCS})
target_link_libraries (${LIBS})
install(TARGETS chanload DESTINATION bin/)
