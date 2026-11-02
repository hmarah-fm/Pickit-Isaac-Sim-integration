FROM rust:1.93.0-alpine3.23 AS builder

RUN apk add --no-cache git cmake make gcc g++ musl-dev linux-headers clang-dev llvm-dev
WORKDIR /src
RUN git init . \
    && git remote add origin https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git \
    && git fetch --depth 1 origin 26e66e1f9867b9faf140fde4e38785508024a554 \
    && git checkout --detach FETCH_HEAD

COPY docker/zenoh/route-lifetime.patch /tmp/route-lifetime.patch
RUN git apply --check /tmp/route-lifetime.patch \
    && git apply /tmp/route-lifetime.patch

# Limit memory consumption when building alongside Isaac Sim.
ENV CARGO_BUILD_JOBS=2 \
    CARGO_PROFILE_RELEASE_LTO=false \
    CARGO_PROFILE_RELEASE_CODEGEN_UNITS=16 \
    CMAKE_POLICY_VERSION_MINIMUM=3.5
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/src/target,sharing=locked \
    cargo build --locked --release -p zenoh-bridge-ros2dds \
    && mkdir -p /out \
    && cp target/release/zenoh-bridge-ros2dds /out/zenoh-bridge-ros2dds

FROM eclipse/zenoh-bridge-ros2dds:1.9.0
COPY --from=builder /out/zenoh-bridge-ros2dds /zenoh-bridge-ros2dds
LABEL org.opencontainers.image.description="Zenoh 1.9.0 with route-owned matching listeners (upstream issue 722)"
