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

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS wlan.proto)

add_library(protobufwlan SHARED ${PROTO_SRCS} ${PROTO_HDRS})

install(TARGETS protobufwlan LIBRARY DESTINATION lib)
