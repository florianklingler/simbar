#!/bin/bash
make cleanall
./configure --veins=../../lib/veins --etsimsg=../../lib/etsi-messages && make -j5
