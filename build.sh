#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Clone RPL4 if it doesn't exist
if [ ! -d "RPL4" ]; then
    echo "Cloning RPL4..."
    git clone https://github.com/yutatech/RPL4.git
fi

# Create .build directory if it doesn't exist
if [ ! -d ".build" ]; then
    echo "Creating .build directory..."
    mkdir .build
fi

cd .build

# Initialize CMake
echo "Configuring CMake..."
cmake .. -DBUILD_EXAMPLES=ON

# Build
echo "Building..."
cmake --build .

echo "Build complete!"
