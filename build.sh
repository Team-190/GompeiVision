#!/bin/bash
set -e

# Detect architecture
arch=$(uname -m)
triplet=""

if [ "$arch" == "x86_64" ]; then
  triplet="x64-linux"
elif [ "$arch" == "aarch64" ]; then
  triplet="arm64-linux"
else
  echo "❌ Unsupported architecture: $arch"
  exit 1
fi

# Set up build directory
mkdir -p build
cmake -S . -B build -G Ninja \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j$(nproc)
