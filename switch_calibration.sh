#!/bin/bash
# Script to switch between different calibration parameter folders

STITCHING_ROOT="/home/koko/Desktop/Projects/foot/stitch/stitching"
ACTIVE_PARAMS_DIR="${STITCHING_ROOT}/params"

if [ -z "$1" ]; then
    echo "Usage: $0 <calibration_folder>"
    echo ""
    echo "Available calibrations:"
    for folder in "${STITCHING_ROOT}"/params*/; do
        if [ -d "$folder" ] && [ -f "${folder}/camchain_0.yaml" ]; then
            folder_name=$(basename "$folder")
            echo "  - $folder_name"
        fi
    done
    echo ""
    echo "Current active calibration (params/):"
    if [ -f "${ACTIVE_PARAMS_DIR}/camchain_0.yaml" ]; then
        echo "  Focal length: $(grep "^focal:" "${ACTIVE_PARAMS_DIR}/camchain_0.yaml")"
        echo "  Distortion D: $(grep "^D:" "${ACTIVE_PARAMS_DIR}/camchain_0.yaml")"
    else
        echo "  No active calibration (params/camchain_0.yaml not found)"
    fi
    exit 1
fi

CALIB_FOLDER="$1"

# Handle both relative and absolute paths, and folder names
if [ -d "${CALIB_FOLDER}" ]; then
    SOURCE_DIR="${CALIB_FOLDER}"
elif [ -d "${STITCHING_ROOT}/${CALIB_FOLDER}" ]; then
    SOURCE_DIR="${STITCHING_ROOT}/${CALIB_FOLDER}"
else
    echo "Error: Calibration folder '${CALIB_FOLDER}' not found!"
    echo ""
    echo "Available calibrations:"
    for folder in "${STITCHING_ROOT}"/params*/; do
        if [ -d "$folder" ] && [ -f "${folder}/camchain_0.yaml" ]; then
            folder_name=$(basename "$folder")
            echo "  - $folder_name"
        fi
    done
    exit 1
fi

# Check if files exist in source
if [ ! -f "${SOURCE_DIR}/camchain_0.yaml" ] || [ ! -f "${SOURCE_DIR}/camchain_1.yaml" ]; then
    echo "Error: Required files (camchain_0.yaml, camchain_1.yaml) not found in '${SOURCE_DIR}'!"
    exit 1
fi

echo "Switching to '$(basename ${SOURCE_DIR})' calibration parameters..."
cp "${SOURCE_DIR}/camchain_0.yaml" "${ACTIVE_PARAMS_DIR}/camchain_0.yaml"
cp "${SOURCE_DIR}/camchain_1.yaml" "${ACTIVE_PARAMS_DIR}/camchain_1.yaml"
echo "✓ Switched to '$(basename ${SOURCE_DIR})' parameters"
echo ""
echo "Active calibration (params/):"
grep "^focal:" "${ACTIVE_PARAMS_DIR}/camchain_0.yaml"
grep "^D:" "${ACTIVE_PARAMS_DIR}/camchain_0.yaml"
