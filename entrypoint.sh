#!/bin/bash

# Entrypoint for AWS Batch
# Downloads videos from S3, runs stitching, uploads result to S3
#
# Expected environment variables (set by Lambda containerOverrides):
#   MATCH_ID        - UUID of the match
#   VIDEO1_S3_PATH  - s3://football-raw-videos/match_{uuid}/video1.mp4
#   VIDEO2_S3_PATH  - s3://football-raw-videos/match_{uuid}/video2.mp4
#   OUTPUT_S3_PATH  - s3://football-stitched-videos/match_{uuid}/output.mp4

set -e

echo "=== Vista Stitching Batch Job ==="
echo "  MATCH_ID:       ${MATCH_ID}"
echo "  VIDEO1_S3_PATH: ${VIDEO1_S3_PATH}"
echo "  VIDEO2_S3_PATH: ${VIDEO2_S3_PATH}"
echo "  OUTPUT_S3_PATH: ${OUTPUT_S3_PATH}"
echo ""

# Validate required environment variables
for var in MATCH_ID VIDEO1_S3_PATH VIDEO2_S3_PATH OUTPUT_S3_PATH; do
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

# Run stitching
echo "Starting stitching..."
/app/run.sh "${INPUT_DIR}/video1.mp4" "${INPUT_DIR}/video2.mp4" "${OUTPUT_DIR}" "output"

# Upload result to S3
RESULT_FILE="${OUTPUT_DIR}/output.mp4"

if [ -f "${RESULT_FILE}" ]; then
    echo ""
    echo "Uploading result to S3..."
    aws s3 cp "${RESULT_FILE}" "${OUTPUT_S3_PATH}"
    echo "Upload complete: ${OUTPUT_S3_PATH}"
else
    echo "Error: stitching output file not found at ${RESULT_FILE}"
    exit 1
fi

echo ""
echo "=== Job complete ==="
