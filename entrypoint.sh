#!/bin/bash

# Entrypoint for AWS Batch
# Downloads videos from S3, runs stitching, uploads MP4 + HLS segments to S3
#
# Expected environment variables (set by Lambda containerOverrides):
#   MATCH_ID            - UUID of the match
#   VIDEO1_S3_PATH      - s3://football-raw-videos/match_{uuid}/video1.mp4
#   VIDEO2_S3_PATH      - s3://football-raw-videos/match_{uuid}/video2.mp4
#   MP4_S3_PATH         - s3://football-stitched-videos/match_{uuid}/match_{uuid}_stitched.mp4
#   HLS_S3_PREFIX       - s3://vista-stream-segments/match_{uuid}

set -e

echo "=== Vista Stitching Batch Job ==="
echo "  MATCH_ID:         ${MATCH_ID}"
echo "  VIDEO1_S3_PATH:   ${VIDEO1_S3_PATH}"
echo "  VIDEO2_S3_PATH:   ${VIDEO2_S3_PATH}"
echo "  MP4_S3_PATH:      ${MP4_S3_PATH}"
echo "  HLS_S3_PREFIX:    ${HLS_S3_PREFIX}"
echo ""

# Validate required environment variables
for var in MATCH_ID VIDEO1_S3_PATH VIDEO2_S3_PATH MP4_S3_PATH HLS_S3_PREFIX; do
    if [ -z "${!var}" ]; then
        echo "Error: ${var} is not set"
        exit 1
    fi
done

# Create working directories
WORK_DIR="/tmp/stitching"
INPUT_DIR="${WORK_DIR}/input"
OUTPUT_DIR="${WORK_DIR}/output"
mkdir -p "${INPUT_DIR}" "${OUTPUT_DIR}"

# Download videos from S3
echo "Downloading video 1..."
aws s3 cp "${VIDEO1_S3_PATH}" "${INPUT_DIR}/video1.mp4"

echo "Downloading video 2..."
aws s3 cp "${VIDEO2_S3_PATH}" "${INPUT_DIR}/video2.mp4"

echo "Downloads complete."
echo ""

# Run stitching (produces MP4 + HLS segments in OUTPUT_DIR/)
echo "Starting stitching + encoding..."
/app/run.sh "${INPUT_DIR}/video1.mp4" "${INPUT_DIR}/video2.mp4" "${OUTPUT_DIR}" "output"

# --- Upload MP4 to football-stitched-videos ---
MP4_FILE="${OUTPUT_DIR}/output.mp4"

if [ -f "${MP4_FILE}" ]; then
    echo ""
    echo "Uploading MP4 to S3..."
    aws s3 cp "${MP4_FILE}" "${MP4_S3_PATH}"
    echo "Upload complete: ${MP4_S3_PATH}"
else
    echo "Error: MP4 output not found at ${MP4_FILE}"
    exit 1
fi

# --- Upload HLS segments to vista-stream-segments ---
HLS_DIR="${OUTPUT_DIR}/output"

if [ -f "${HLS_DIR}/master.m3u8" ]; then
    echo ""
    SEGMENT_COUNT=$(ls -1 "${HLS_DIR}"/*.ts 2>/dev/null | wc -l)
    echo "Uploading HLS playlist + ${SEGMENT_COUNT} segments to S3..."
    aws s3 sync "${HLS_DIR}" "${HLS_S3_PREFIX}" \
        --content-type "application/vnd.apple.mpegurl" \
        --exclude "*" --include "*.m3u8"
    aws s3 sync "${HLS_DIR}" "${HLS_S3_PREFIX}" \
        --content-type "video/MP2T" \
        --exclude "*" --include "*.ts"
    echo "Upload complete: ${HLS_S3_PREFIX}/master.m3u8"
else
    echo "Error: HLS output not found at ${HLS_DIR}/master.m3u8"
    ls -la "${OUTPUT_DIR}" 2>/dev/null || true
    ls -la "${HLS_DIR}" 2>/dev/null || true
    exit 1
fi

echo ""
echo "=== Job complete ==="
