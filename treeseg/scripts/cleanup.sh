#!/bin/bash
set -e

BUILD_DIR="/home/ubuntu/treeseg_june26_2025/treeseg/build"

echo "Cleaning up old binaries..."

# Remove old executables if they exist
rm -f $BUILD_DIR/downsample
rm -f $BUILD_DIR/findstems
rm -f $BUILD_DIR/getcrownvolume
rm -f $BUILD_DIR/getdtmslice
rm -f $BUILD_DIR/nearestneighbour
rm -f $BUILD_DIR/pcdPointTreeseg2txt
rm -f $BUILD_DIR/pcdPointXYZRGB2txt
rm -f $BUILD_DIR/plotcoords
rm -f $BUILD_DIR/segmentcrown
rm -f $BUILD_DIR/segmentstem
rm -f $BUILD_DIR/sepwoodleaf
rm -f $BUILD_DIR/thin
rm -f $BUILD_DIR/txtPointTreeseg2pcd

# Remove old shared libraries
rm -f $BUILD_DIR/*.so

echo "Cleanup complete!"