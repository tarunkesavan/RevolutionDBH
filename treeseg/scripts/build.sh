#!/bin/bash
set -e

# Move to project folder
cd $(dirname $0)/..

# Create build folder
mkdir -p build
cd build

# Run CMake
cmake ..
make -j$(nproc)

# Copy binaries to a deploy folder for CodeDeploy
mkdir -p ../deploy
cp my_binary ../deploy/     # replace my_binary with actual binary names
