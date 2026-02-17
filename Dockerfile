# Use Ubuntu as base image with OpenCV dependencies pre-installed
FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    opencv-data \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy the project files
COPY . .

# Make the wrapper script executable
RUN chmod +x run.sh

# Create build directory and build the project
RUN mkdir -p build && \
    cd build && \
    cmake .. && \
    make

# Set the entrypoint to the wrapper script
ENTRYPOINT ["./run.sh"]
