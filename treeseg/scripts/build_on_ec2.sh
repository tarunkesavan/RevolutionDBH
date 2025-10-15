#!/bin/bash
set -e

TREESEG_DIR="/home/ubuntu/treeseg_june26_2025/treeseg"

echo "============================================"
echo "  Installing build dependencies on EC2     "
echo "============================================"

# Install build tools and libraries
sudo apt-get update
sudo apt-get install -y build-essential cmake python3-dev
sudo apt-get install -y libpcl-dev libarmadillo-dev libxlsxwriter-dev

echo ""
echo "============================================"
echo "  Configuring CMakeLists.txt for Ubuntu    "
echo "============================================"

cd $TREESEG_DIR

# Update C++ standard to C++17 (required for std::filesystem)
sed -i 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 17)/g' CMakeLists.txt

# Get Python paths dynamically
PYTHON_INCLUDE=$(python3 -c "import sysconfig; print(sysconfig.get_path('include'))")
PYTHON_LIB=$(python3 -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
PYTHON_LIBNAME=$(python3 -c "import sysconfig; print(f\"libpython{sysconfig.get_config_var('LDVERSION')}.so\")")

echo "Python include: $PYTHON_INCLUDE"
echo "Python lib: $PYTHON_LIB/$PYTHON_LIBNAME"

# Update Python paths from macOS to Ubuntu
sed -i "s|set(Python3_ROOT_DIR \"/Library/Frameworks/Python.framework/Versions/3.11\")|set(Python3_ROOT_DIR \"/usr\")|g" CMakeLists.txt
sed -i "s|/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11|$PYTHON_INCLUDE|g" CMakeLists.txt
sed -i "s|/Library/Frameworks/Python.framework/Versions/3.11/lib/libpython3.11.dylib|$PYTHON_LIB/$PYTHON_LIBNAME|g" CMakeLists.txt

echo ""
echo "============================================"
echo "  Building project with CMake              "
echo "============================================"

# Create and enter build directory
mkdir -p build
cd build

# Run CMake
cmake ..

# Build with all available cores
make -j$(nproc)

echo ""
echo "============================================"
echo "  Build Complete!                          "
echo "============================================"
echo ""
echo "Built executables:"
ls -lh | grep -E '^-rwx'
echo ""
echo "Built libraries:"
ls -lh *.so 2>/dev/null || echo "No shared libraries"
echo ""