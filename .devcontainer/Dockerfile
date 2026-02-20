FROM ubuntu:latest

# Avoid prompts during install
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
&& apt-get install -y --no-install-recommends python3 \
python3-pip \
build-essential \
git \
python3-dev \
cmake \
ninja-build \
llvm \
pkg-config \
libfmt-dev \
lsb-release \
curl \
&& rm -rf /var/lib/apt/lists/*

RUN python3 -m pip config set global.break-system-packages true
RUN pip install --no-cache-dir uv maturin ruff