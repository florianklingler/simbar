#!/bin/bash

set -e

source export.sh

# Add dependencies as git subtrees
echo "********** Add dependencies as git subtrees **********"
git subtree add --prefix=lib/veins https://github.com/sommer/veins veins-5.0 --squash
git subtree add --prefix=lib/sumo https://github.com/eclipse/sumo v1_2_0 --squash
git subtree add --prefix=lib/omnetpp https://github.com/omnetpp/omnetpp omnetpp-5.5.1 --squash
git subtree add --prefix=lib/openWRT https://github.com/openwrt/openwrt v19.07.2 --squash
git subtree add --prefix=lib/asn1c https://github.com/zhanglei002/asn1c master --squash

# build asn1c compiler
echo "********** Build asn1c compiler **********"
cd lib/asn1c
autoreconf -iv && ./configure
make
cd ../..

# Get ETSI ITS G5 asn1 files
echo "********** Get ETSI ITS G5 asn1 files **********"
cd lib/etsi-messages
make
cd ../..

# Apply SimbaR patch
echo "********** Apply SimbaR patch **********"
git apply SimbaR.patch

# Build ETSI ITS G5 asn1 files
echo "********** Build ETSI ITS G5 asn1 files **********"
cd lib/etsi-messages
make
cd ../..

# Build OMNeT++
echo "********** Build OMNeT++ **********"
cd lib/omnetpp
cp configure.user.dist configure.user
sed -i '/WITH_OSG=yes/c\WITH_OSG=no' configure.user
sed -i '/WITH_OSGEARTH=yes/c\WITH_OSGEARTH=no' configure.user
./configure
make -j4
cd ../..

# Build SUMO
echo "********** Build SUMO **********"
cd lib/sumo
mkdir  -p build/cmake-build && cd build/cmake-build
cmake ../..
make -j4
cd ../../../..

#Build Veins
echo "********** Build Veins **********"
cd lib/veins
./configure && make -j4
cd ../..

#Build SimbaR extension for Veins
echo "********** Build Simbar extension for Veins **********"
cd src/SimbaR/src/SimbaR
# Download pstreams dependency
wget https://downloads.sourceforge.net/project/pstreams/pstreams/Release%201.0/pstreams-1.0.1.tar.gz
tar -xf pstreams-1.0.1.tar.gz
rm pstreams-1.0.1.tar.gz
mv pstreams-1.0.1/pstream.h .
rm -r pstreams-1.0.1
cd ../..
./compile.sh
cd ../..

#Build OpenWrt
echo "********** Build OpenWrt **********"
# Download ZeroMQ dependency
mkdir -p src/openWRT-feed/lanradio/src/zmq && cd src/openWRT-feed/lanradio/src/zmq
wget https://github.com/zeromq/cppzmq/raw/v4.2.2/zmq.hpp
cd ../../../../..
# Download pstreams dependency
mkdir -p src/openWRT-feed/lanradio/src/pstreams && cd src/openWRT-feed/lanradio/src/pstreams
wget https://downloads.sourceforge.net/project/pstreams/pstreams/Release%201.0/pstreams-1.0.1.tar.gz
tar -xf pstreams-1.0.1.tar.gz
rm pstreams-1.0.1.tar.gz
mv pstreams-1.0.1/* .
rm -r pstreams-1.0.1
cd ../../../../../lib/openWRT
# Update openWRT feeds
./scripts/feeds update -a
./scripts/feeds install -a
# Copy config file at the right position
make defconfig	#create .config
cat ../../SimbaR_diff.config >> .config
make defconfig	#apply SimbaR config
# Call make
VERSION="0.01"
BUILD_DATE="$(date --utc)"
make package/base-files/clean
make -j4 CONFIG_VERSION_CODE="$VERSION - built $BUILD_DATE"
cd ../..
