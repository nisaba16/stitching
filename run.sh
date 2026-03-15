#!/bin/bash

# Wrapper script for image-stitching that auto-computes FPS

set -e

if [ $# -lt 2 ]; then
    echo "Usage: $0 <video_file1> <video_file2> [output_folder] [output_name]"
    echo ""
    echo "Examples:"
    echo "  $0 /videos/left.mp4 /videos/right.mp4"
    echo "  $0 /videos/left.mp4 /videos/right.mp4 /output stitched"
    echo ""
    echo "Defaults:"
    echo "  - output_folder: /output"
    echo "  - output_name: stitched_video"
    echo "  - FPS: auto-detected from first video"
    echo "  - dry_run: false"
    echo "  - use_lir: false"
    echo "  - use_calibration: true (if params/ exists, else false)"
    echo "  - use_sift: false"
    exit 1
fi

VIDEO1="$1"
VIDEO2="$2"
OUTPUT_FOLDER="${3:-/output}"
OUTPUT_NAME="${4:-stitched_video}"

# Check if videos exist
if [ ! -f "$VIDEO1" ]; then
    echo "Error: Video file not found: $VIDEO1"
    exit 1
fi

if [ ! -f "$VIDEO2" ]; then
    echo "Error: Video file not found: $VIDEO2"
    exit 1
fi

# Auto-detect FPS from first video using ffprobe
FPS=$(ffprobe -v error -select_streams v:0 -show_entries stream=r_frame_rate -of default=noprint_wrappers=1:nokey=1:noesc=1 "$VIDEO1" | awk -F'/' '{printf "%.0f", $1/$2}')

if [ -z "$FPS" ] || [ "$FPS" -eq 0 ]; then
    echo "Warning: Could not detect FPS, using default 30"
    FPS=30
fi

echo "Detected FPS: $FPS"

# Check if calibration files exist
USE_CALIBRATION="false"
if [ -d "/app/params" ] && [ -f "/app/params/camchain_0.yaml" ]; then
    USE_CALIBRATION="true"
    echo "Calibration files found, using calibration"
else
    echo "No calibration files found, using on-the-fly calibration"
fi

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_FOLDER"

echo "Running image stitching..."
echo "  Video 1: $VIDEO1"
echo "  Video 2: $VIDEO2"
echo "  Output: $OUTPUT_FOLDER/$OUTPUT_NAME.mp4 + $OUTPUT_FOLDER/$OUTPUT_NAME/ (HLS)"
echo "  FPS: $FPS"
echo "  Calibration: $USE_CALIBRATION"

# Run the stitching application
# Parameters: output_folder, file_name, fps, dry_run, use_lir, use_calibration, use_sift, use_feature_mask, video_file1, video_file2
/app/build/image-stitching "$OUTPUT_FOLDER" "$OUTPUT_NAME" "$FPS" false false "$USE_CALIBRATION" false false "$VIDEO1" "$VIDEO2"

MP4_FILE="$OUTPUT_FOLDER/$OUTPUT_NAME.mp4"
HLS_DIR="$OUTPUT_FOLDER/$OUTPUT_NAME"

# Check if stitching was successful
if [ ! -f "$MP4_FILE" ]; then
    echo "Error: Stitching failed - MP4 output not created at $MP4_FILE"
    exit 1
fi

if [ ! -f "$HLS_DIR/master.m3u8" ]; then
    echo "Error: Stitching failed - HLS output not created at $HLS_DIR/master.m3u8"
    exit 1
fi

MP4_SIZE=$(du -h "$MP4_FILE" | cut -f1)
SEGMENT_COUNT=$(ls -1 "$HLS_DIR"/*.ts 2>/dev/null | wc -l)
echo "Stitching complete!"
echo "  MP4:  $MP4_FILE ($MP4_SIZE)"
echo "  HLS:  $HLS_DIR/master.m3u8 ($SEGMENT_COUNT segments)"
