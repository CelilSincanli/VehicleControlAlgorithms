# ── Stage 1: Build ─────────────────────────────────────────────────────────────
FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j$(nproc)

# ── Stage 2: Runtime ───────────────────────────────────────────────────────────
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglfw3 \
    libgl1-mesa-glx \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# The binary has ASSET_PATH, DATA_PATH, and CONFIG_PATH baked in at compile time
# as absolute paths relative to CMAKE_SOURCE_DIR (/app). The runtime stage must
# preserve the same layout so those paths resolve correctly.
COPY --from=builder /app/build/vehicle_control_algorithms ./vehicle_control_algorithms
COPY --from=builder /app/include/assets                   ./include/assets
COPY --from=builder /app/data                             ./data
COPY --from=builder /app/config                           ./config

CMD ["./vehicle_control_algorithms"]
