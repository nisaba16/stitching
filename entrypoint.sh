#!/bin/bash

# Entrypoint for AWS Batch
# Downloads video parts from an S3 folder, concatenates cam_A (left) and cam_B (right)
# parts in order, stitches them, and uploads MP4 + HLS segments to S3.
#
# Expected environment variables (set by Lambda containerOverrides):
#   MATCH_ID            - UUID of the match
#   INPUT_S3_PREFIX     - s3://football-raw-videos/match_{uuid}/  (folder containing
#                           cam_A_<date>_<hour>_part000.mp4 ... and
#                           cam_B_<date>_<hour>_part000.mp4 ... )
#   MP4_S3_PATH         - s3://football-stitched-videos/match_{uuid}/match_{uuid}_stitched.mp4
#   HLS_S3_PREFIX       - s3://vista-stream-segments/match_{uuid}

set -e

echo "=== Vista Stitching Batch Job ==="
echo "  MATCH_ID:         ${MATCH_ID}"
echo "  INPUT_S3_PREFIX:  ${INPUT_S3_PREFIX}"
echo "  MP4_S3_PATH:      ${MP4_S3_PATH}"
echo "  HLS_S3_PREFIX:    ${HLS_S3_PREFIX}"
echo ""

# Validate required environment variables
for var in MATCH_ID INPUT_S3_PREFIX MP4_S3_PATH HLS_S3_PREFIX; do
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

# Download all cam_A and cam_B parts from the S3 folder
echo "Downloading video parts from ${INPUT_S3_PREFIX}..."
aws s3 sync "${INPUT_S3_PREFIX}" "${INPUT_DIR}" \
    --exclude "*" --include "cam_A_*.mp4" --include "cam_B_*.mp4"
echo "Downloads complete."
echo ""

# Collect files per camera, sorted by name (guarantees part000 < part001 < ...)
mapfile -t CAM_A_FILES < <(ls "${INPUT_DIR}"/cam_A_*.mp4 2>/dev/null | sort)
mapfile -t CAM_B_FILES < <(ls "${INPUT_DIR}"/cam_B_*.mp4 2>/dev/null | sort)

if [ ${#CAM_A_FILES[@]} -eq 0 ]; then
    echo "Error: no cam_A_*.mp4 files found in ${INPUT_DIR}"
    ls -la "${INPUT_DIR}"
    exit 1
fi
if [ ${#CAM_B_FILES[@]} -eq 0 ]; then
    echo "Error: no cam_B_*.mp4 files found in ${INPUT_DIR}"
    ls -la "${INPUT_DIR}"
    exit 1
fi

echo "cam_A: ${#CAM_A_FILES[@]} part(s)  |  cam_B: ${#CAM_B_FILES[@]} part(s)"

# Helper: concatenate an array of video files into a single output file.
# Skips FFmpeg when there is only one part.
concat_parts() {
    local output_file="$1"
    shift
    local parts=("$@")

    if [ ${#parts[@]} -eq 1 ]; then
        # Single part — use directly without re-muxing
        echo "${parts[0]}"
        return
    fi

    local list_file="${WORK_DIR}/$(basename "${output_file}" .mp4)_list.txt"
    printf "file '%s'\n" "${parts[@]}" > "${list_file}"
    ffmpeg -y -f concat -safe 0 -i "${list_file}" -c copy "${output_file}" \
        -loglevel warning -stats
    echo "${output_file}"
}

echo ""
echo "Preparing cam_A (left)..."
CAM_A_INPUT=$(concat_parts "${WORK_DIR}/cam_A.mp4" "${CAM_A_FILES[@]}")
echo "cam_A ready: ${CAM_A_INPUT}"

echo "Preparing cam_B (right)..."
CAM_B_INPUT=$(concat_parts "${WORK_DIR}/cam_B.mp4" "${CAM_B_FILES[@]}")
echo "cam_B ready: ${CAM_B_INPUT}"

echo ""

# Run stitching (produces MP4 + HLS segments in OUTPUT_DIR/)
echo "Starting stitching + encoding..."
/app/run.sh "${CAM_A_INPUT}" "${CAM_B_INPUT}" "${OUTPUT_DIR}" "output"

# --- Upload MP4 ---
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

# --- Upload HLS segments ---
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
