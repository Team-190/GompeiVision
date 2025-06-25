# Use an Ubuntu base image
FROM ubuntu:24.04

# Install essential packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    curl \
    unzip \
    ca-certificates \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set working directory inside the container
WORKDIR /workspace

# Copy your entire repo into the container (adjust as needed)
COPY . /workspace

# Download and bootstrap vcpkg if not already present
RUN if [ ! -d /vcpkg ]; then \
      git clone https://github.com/microsoft/vcpkg.git /vcpkg && \
      /vcpkg/bootstrap-vcpkg.sh; \
    fi

# Build argument to specify target architecture
ARG TARGET_ARCH=amd64

# Map TARGET_ARCH to vcpkg triplet
RUN if [ "$TARGET_ARCH" = "amd64" ]; then \
      TRIPLET=x64-linux; \
    elif [ "$TARGET_ARCH" = "arm64" ]; then \
      TRIPLET=arm64-linux; \
    else \
      echo "Unknown arch: $TARGET_ARCH"; exit 1; \
    fi && \
    /vcpkg/vcpkg install --triplet $TRIPLET && \
    cmake -S /workspace -B /workspace/build -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=/vcpkg/scripts/buildsystems/vcpkg.cmake \
      -DVCPKG_TARGET_TRIPLET=$TRIPLET \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /workspace/build -- -j$(nproc)

# The built binary will be inside /workspace/build/
