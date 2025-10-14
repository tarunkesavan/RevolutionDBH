#!/bin/bash
set -e

# Copy built files to EC2 folder
cp -r ../deploy/* /home/ubuntu/treeseg_june26_2025/

# Make binaries executable
chmod +x /home/ubuntu/treeseg_june26_2025/*
