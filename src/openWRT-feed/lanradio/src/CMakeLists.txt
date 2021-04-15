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

project(lanradio C CXX)

SET(PROJECT_VERSION_MAJOR "1")
SET(PROJECT_VERSION_MINOR "0")
SET(CMAKE_CXX_FLAGS "-g -Ofast -std=c++11 -fpermissive -pthread")


find_package(Boost COMPONENTS program_options)

include_directories(
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
)

# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS protobuf/wlan.proto)


set(lanradio_SRCS
   lanradio.cc
)

add_executable (lanradio ${lanradio_SRCS} ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries (lanradio pthread boost_program_options zmq protobuf tins ${Protobuf_LIBRARIES})
install(TARGETS lanradio DESTINATION bin/)


