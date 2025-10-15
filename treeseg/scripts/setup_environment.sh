#!/bin/bash
set -e

BUILD_DIR="/home/ubuntu/treeseg_june26_2025/treeseg/build"

echo "============================================"
echo "  Setting up runtime environment           "
echo "============================================"

# Add library path to .bashrc if not already there
if ! grep -q "LD_LIBRARY_PATH.*treeseg" /home/ubuntu/.bashrc; then
    echo "" >> /home/ubuntu/.bashrc
    echo "# Treeseg library path (auto-added by deployment)" >> /home/ubuntu/.bashrc
    echo "export LD_LIBRARY_PATH=$BUILD_DIR:\$LD_LIBRARY_PATH" >> /home/ubuntu/.bashrc
    echo "✓ Library path added to .bashrc"
else
    echo "✓ Library path already configured in .bashrc"
fi

# Create a convenience script to run treeseg commands
cat > /home/ubuntu/run_treeseg.sh << 'EOF'
#!/bin/bash
# Convenience script to run treeseg executables with correct library path
export LD_LIBRARY_PATH=/home/ubuntu/treeseg_june26_2025/treeseg/build:$LD_LIBRARY_PATH
cd /home/ubuntu/treeseg_june26_2025/treeseg/build
"$@"
EOF

chmod +x /home/ubuntu/run_treeseg.sh

# Also add treeseg/build to PATH for convenience
if ! grep -q "treeseg_june26_2025/treeseg/build" /home/ubuntu/.bashrc; then
    echo "export PATH=$BUILD_DIR:\$PATH" >> /home/ubuntu/.bashrc
    echo "✓ Added treeseg/build to PATH"
fi

echo ""
echo "============================================"
echo "  Environment setup complete!              "
echo "============================================"
echo ""
echo "To use treeseg commands:"
echo "  1. Log out and back in (for PATH changes)"
echo "  2. Or run: source ~/.bashrc"
echo "  3. Or use: ~/run_treeseg.sh ./downsample <args>"
echo ""