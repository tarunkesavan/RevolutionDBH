#!/bin/bash
set -e

TARGET_DIR="/home/ubuntu/treeseg_june26_2025/treeseg"

echo "Cleaning up old deployment at $TARGET_DIR..."
if [ -d "$TARGET_DIR" ]; then
    rm -rf "$TARGET_DIR"
    echo "Old deployment removed."
fi

echo "Cleanup complete!"
