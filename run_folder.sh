#!/bin/bash

# Stitch all cam_A / cam_B parts from a local input folder.
# Parts are concatenated (stream copy) then passed to the stitching pipeline.
#
# Usage:
#   ./run_folder.sh <input_folder> [results_base]
#
# Output:
#   <results_base>/<folder_name>/stitched.mp4
#   <results_base>/<folder_name>/stitched/master.m3u8  + *.ts

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <input_folder> [results_base]"
    echo ""
    echo "Examples:"
    echo "  $0 datasets/Montrouge_FC_Chaville_FC_10-05-26"
    echo "  $0 /data/matches/Montrouge_FC_Chaville_FC_10-05-26 /mnt/results"
    echo ""
    echo "Defaults:"
    echo "  results_base: <script_dir>/results"
    exit 1
fi

INPUT_FOLDER="$(cd "$1" && pwd)"
FOLDER_NAME="$(basename "$INPUT_FOLDER")"
RESULTS_BASE="${2:-${SCRIPT_DIR}/results}"
OUTPUT_DIR="${RESULTS_BASE}/${FOLDER_NAME}"

BINARY="${SCRIPT_DIR}/build/image-stitching"
if [ ! -f "$BINARY" ]; then
    echo "Error: binary not found at ${BINARY} — run 'cmake .. && make' in build/"
    exit 1
fi

echo "=== Folder Stitching ==="
echo "  Input:   ${INPUT_FOLDER}"
echo "  Output:  ${OUTPUT_DIR}"
echo ""

# Collect parts for each camera, sorted by filename (part000 < part001 < ...)
mapfile -t CAM_A_FILES < <(ls "${INPUT_FOLDER}"/*cam_A*part*.mp4 2>/dev/null | sort)
mapfile -t CAM_B_FILES < <(ls "${INPUT_FOLDER}"/*cam_B*part*.mp4 2>/dev/null | sort)

if [ ${#CAM_A_FILES[@]} -eq 0 ]; then
    echo "Error: no *cam_A*part*.mp4 files found in ${INPUT_FOLDER}"
    exit 1
fi
if [ ${#CAM_B_FILES[@]} -eq 0 ]; then
    echo "Error: no *cam_B*part*.mp4 files found in ${INPUT_FOLDER}"
    exit 1
fi

echo "cam_A: ${#CAM_A_FILES[@]} part(s)  |  cam_B: ${#CAM_B_FILES[@]} part(s)"

WORK_DIR="/tmp/stitching_${FOLDER_NAME}"
mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

# Concatenate one camera's parts into a single file (stream copy, no re-encoding).
# Skips FFmpeg when there is only one part.
concat_parts() {
    local output_file="$1"
    shift
    local parts=("$@")

    if [ ${#parts[@]} -eq 1 ]; then
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

echo "Preparing cam_B (right)..."
CAM_B_INPUT=$(concat_parts "${WORK_DIR}/cam_B.mp4" "${CAM_B_FILES[@]}")

# Auto-detect FPS from cam_A
FPS=$(ffprobe -v error -select_streams v:0 \
    -show_entries stream=r_frame_rate \
    -of default=noprint_wrappers=1:nokey=1:noesc=1 \
    "${CAM_A_INPUT}" | awk -F'/' '{printf "%.0f", $1/$2}')
if [ -z "$FPS" ] || [ "$FPS" -eq 0 ]; then
    echo "Warning: could not detect FPS, defaulting to 30"
    FPS=30
fi

# Check for calibration files
USE_CALIBRATION="false"
if [ -f "${SCRIPT_DIR}/params/camchain_0.yaml" ]; then
    USE_CALIBRATION="true"
fi

echo ""
echo "Running stitching..."
echo "  Left:  ${CAM_A_INPUT}"
echo "  Right: ${CAM_B_INPUT}"
echo "  FPS:   ${FPS}  |  Calibration: ${USE_CALIBRATION}"
echo "  Out:   ${OUTPUT_DIR}/stitched.mp4 + ${OUTPUT_DIR}/stitched/ (HLS)"
echo ""

"${BINARY}" "${OUTPUT_DIR}" "stitched" "${FPS}" \
    false false "${USE_CALIBRATION}" false false \
    "${CAM_A_INPUT}" "${CAM_B_INPUT}"

# Verify outputs
MP4_FILE="${OUTPUT_DIR}/stitched.mp4"
HLS_DIR="${OUTPUT_DIR}/stitched"

if [ ! -f "${MP4_FILE}" ]; then
    echo "Error: MP4 not created at ${MP4_FILE}"
    exit 1
fi
if [ ! -f "${HLS_DIR}/master.m3u8" ]; then
    echo "Error: HLS playlist not created at ${HLS_DIR}/master.m3u8"
    exit 1
fi

MP4_SIZE=$(du -h "${MP4_FILE}" | cut -f1)
SEGMENT_COUNT=$(ls -1 "${HLS_DIR}"/*.ts 2>/dev/null | wc -l)
echo ""
echo "=== Done ==="
echo "  MP4:  ${MP4_FILE} (${MP4_SIZE})"
echo "  HLS:  ${HLS_DIR}/master.m3u8 (${SEGMENT_COUNT} × 10 s segments)"
