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
echo "  Output: $OUTPUT_FOLDER/$OUTPUT_NAME.mp4"
echo "  FPS: $FPS"
echo "  Calibration: $USE_CALIBRATION"

# Run the stitching application
# Parameters: output_folder, file_name, fps, dry_run, use_lir, use_calibration, use_sift, use_feature_mask, video_file1, video_file2
/app/build/image-stitching "$OUTPUT_FOLDER" "$OUTPUT_NAME" "$FPS" false false "$USE_CALIBRATION" false false "$VIDEO1" "$VIDEO2"

RESULT_FILE="$OUTPUT_FOLDER/$OUTPUT_NAME.mp4"

# Check if stitching was successful
if [ -f "$RESULT_FILE" ]; then
    echo "Stitching complete! Output file: $RESULT_FILE"
    
    # Upload to S3 if AWS credentials are provided
    if [ -n "$AWS_ACCESS_KEY_ID" ] && [ -n "$AWS_SECRET_ACCESS_KEY" ] && [ -n "$MATCH_UUID" ]; then
        echo "Uploading to S3..."
        S3_PATH="s3://football-stitched-videos/match_${MATCH_UUID}/output.mp4"
        
        aws s3 cp "$RESULT_FILE" "$S3_PATH"
        
        if [ $? -eq 0 ]; then
            echo "Successfully uploaded to: $S3_PATH"
        else
            echo "Failed to upload to S3"
            exit 1
        fi
    else
        echo "S3 upload skipped - missing AWS credentials or MATCH_UUID"
        echo "To upload to S3, provide: AWS_ACCESS_KEY_ID, AWS_SECRET_ACCESS_KEY, MATCH_UUID"
    fi
else
    echo "Error: Stitching failed - output file not created"
    exit 1
fi
