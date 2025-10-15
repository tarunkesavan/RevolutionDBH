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
mkdir -p ../deploy/bin
mkdir -p ../deploy/lib

# Copy all executables
cp findstems ../deploy/bin/ || echo "findstems not found"
cp downsample ../deploy/bin/ || echo "downsample not found"
cp getdtmslice ../deploy/bin/ || echo "getdtmslice not found"
cp nearestneighbour ../deploy/bin/ || echo "nearestneighbour not found"
cp plotcoords ../deploy/bin/ || echo "plotcoords not found"
cp segmentstem ../deploy/bin/ || echo "segmentstem not found"
cp getcrownvolume ../deploy/bin/ || echo "getcrownvolume not found"
cp segmentcrown ../deploy/bin/ || echo "segmentcrown not found"
cp sepwoodleaf ../deploy/bin/ || echo "sepwoodleaf not found"
cp pcdPointTreeseg2txt ../deploy/bin/ || echo "pcdPointTreeseg2txt not found"
cp txtPointTreeseg2pcd ../deploy/bin/ || echo "txtPointTreeseg2pcd not found"
cp pcdPointXYZRGB2txt ../deploy/bin/ || echo "pcdPointXYZRGB2txt not found"
cp thin ../deploy/bin/ || echo "thin not found"

# Copy shared libraries
cp *.so ../deploy/lib/ 2>/dev/null || echo "No .so files found"

echo "Build complete! Artifacts copied to ../deploy/"
ls -la ../deploy/bin/
ls -la ../deploy/lib/