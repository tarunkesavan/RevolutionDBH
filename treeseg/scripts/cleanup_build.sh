#!/bin/bash
set -e

BUILD_DIR="/home/ubuntu/treeseg_june26_2025/treeseg/build"

echo "Cleaning old build..."
if [ -d "$BUILD_DIR" ]; then
    rm -rf $BUILD_DIR/*
    echo "Old build files removed"
fi

echo "Cleanup complete!"