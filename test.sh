#!/bin/bash
# Script to build test target.
#
# Go into the directory where this bash script is contained.
cd `dirname $0`

#./clean.sh

# Compile code.
mkdir -p build
cmake -S ./tests -B build
cd build
make $*

./bin/particle_filter_tests