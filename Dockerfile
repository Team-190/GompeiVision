ARG TARGET_ARCH=amd64
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y \
    build-essential ninja-build cmake curl git unzip zip tar pkg-config python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY . /workspace

RUN git clone https://github.com/microsoft/vcpkg.git /vcpkg && \
    /vcpkg/bootstrap-vcpkg.sh

ENV PATH="/vcpkg:${PATH}"

# Map Docker TARGET_ARCH to vcpkg triplet (simple example)
RUN if [ "$TARGET_ARCH" = "amd64" ]; then \
      export TRIPLET=x64-linux; \
    elif [ "$TARGET_ARCH" = "arm64" ]; then \
      export TRIPLET=arm64-linux; \
    else \
      echo "Unknown arch"; exit 1; \
    fi && \
    /vcpkg/vcpkg install --triplet $TRIPLET && \
    cmake -S /workspace -B /workspace/build -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=/vcpkg/scripts/buildsystems/vcpkg.cmake \
      -DVCPKG_TARGET_TRIPLET=$TRIPLET \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /workspace/build -- -j$(nproc)

CMD ["ls", "-lh", "/workspace/build"]
