# Stereo Video Stitching

Real-time panoramic video stitching for two side-by-side cameras with stereo calibration support.

## Table of Contents

- [Quick Start](#quick-start)
- [Installation](#installation)
  - [Docker (Recommended)](#docker-recommended)
  - [Local Build](#local-build)
- [Usage](#usage)
  - [Command Line Parameters](#command-line-parameters)
  - [Feature Detectors](#feature-detectors)
  - [Examples](#examples)
- [Local Folder Mode](#local-folder-mode)
  - [Input convention](#input-convention)
  - [Output structure](#output-structure)
- [AWS Batch](#aws-batch)
  - [Input folder convention](#input-folder-convention)
  - [Environment variables](#environment-variables)
  - [Job flow](#job-flow)
- [Calibration](#calibration)
  - [Converting NPY to YAML](#converting-npy-to-yaml)
  - [Switching Calibrations](#switching-calibrations)
  - [Folder Structure](#folder-structure)
- [Debug Outputs](#debug-outputs)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)

---

## Quick Start

**Stitch a local match folder (simplest for local use):**
```bash
mkdir build && cd build && cmake .. && make -j$(nproc) && cd ..
./run_folder.sh datasets/Montrouge_FC_Chaville_FC_10-05-26
# → results/Montrouge_FC_Chaville_FC_10-05-26/stitched.mp4  +  stitched/ (HLS)
```

**With Docker:**
```bash
docker build -t image-stitching:latest .
docker run -v /path/to/videos:/videos -v /path/to/output:/output \
    image-stitching:latest /videos/left.mp4 /videos/right.mp4
```

**Local build (two files):**
```bash
mkdir build && cd build
cmake .. && make -j$(nproc)
./image-stitching ../results stitched 30 true false true AKAZE false ../datasets/left.mp4 ../datasets/right.mp4
```

---

## Installation

### Docker (Recommended)

```bash
docker build -t image-stitching:latest .
```

Run with auto-detected FPS:
```bash
docker run -v /path/to/videos:/videos -v /path/to/output:/output \
    image-stitching:latest /videos/left.mp4 /videos/right.mp4
```

With custom output name:
```bash
docker run -v /path/to/videos:/videos -v /path/to/output:/output \
    image-stitching:latest /videos/left.mp4 /videos/right.mp4 /output my_panorama
```

With calibration files:
```bash
docker run -v $(pwd)/params:/app/params \
           -v /path/to/videos:/videos \
           -v /path/to/output:/output \
           image-stitching:latest /videos/left.mp4 /videos/right.mp4
```

### Local Build

**Requirements:** OpenCV 4.x with contrib modules, CMake 3.10+

```bash
mkdir build && cd build
cmake .. && make -j$(nproc)
```

---

## Usage

```bash
./image-stitching <output_folder> <file_name> <fps> <dry_run> <use_lir> <use_calibration> <feature_method> <use_feature_mask> <video1> <video2>
```

### Command Line Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `output_folder` | path | Directory for output files |
| `file_name` | string | Output filename (without extension) |
| `fps` | int | Output video frame rate |
| `dry_run` | bool | `true` = test first frame only, `false` = process full video |
| `use_lir` | bool | `true` = crop to largest interior rectangle |
| `use_calibration` | bool | `true` = use `params/*.yaml`, `false` = estimate on-the-fly |
| `feature_method` | string | `SIFT`, `AKAZE`, or `ORB` |
| `use_feature_mask` | bool | `true` = detect features only in overlap zone |
| `video1`, `video2` | path | Input video files (left, right) |

### Feature Detectors

| Detector | Speed | Accuracy | Best For |
|----------|-------|----------|----------|
| **AKAZE** | Fast | Good | General use, recommended default |
| **SIFT** | Slow | Excellent | Difficult scenes, high texture variation |
| **ORB** | Very fast | Moderate | Real-time when speed is critical |

The pipeline automatically:
- Applies **CLAHE** contrast enhancement for better matching on uniform surfaces (grass)
- Uses **overlap-aware masking** to focus features in the shared field of view
- Runs **bundle adjustment** with rotation sanity checks to prevent drift

### Examples

**Dry run** (test with first frame, saves preview image):
```bash
./image-stitching ../results test 30 true false true AKAZE false left.mp4 right.mp4
# Output: ../results/dry_run_test.jpg
```

**Full video with calibration:**
```bash
./image-stitching ../results match_stitched 30 false false true AKAZE false \
    ../datasets/stable_0c.mp4 ../datasets/stable_1c.mp4 

./image-stitching ../results match_stitched 30 false false true AKAZE false \
    ../datasets/match_cam0.mp4 ../datasets/match_cam1.mp4
```

**Without calibration** (estimates from features):
```bash
./image-stitching ../results output 30 false false false SIFT false left.mp4 right.mp4
```

**With LIR cropping** (removes black borders):
```bash
./image-stitching ../results output 30 false true true AKAZE false left.mp4 right.mp4
```

---

## Local Folder Mode

`run_folder.sh` is the recommended entry point for local stitching when your input is a folder of multi-part recordings.

### Input convention

```
datasets/Montrouge_FC_Chaville_FC_10-05-26/
├── 14625_cam_A_20260510_0108_part000.mp4   ← left camera, part 0
├── 14625_cam_A_20260510_0108_part001.mp4
├── ...
├── 14625_cam_B_20260510_0108_part000.mp4   ← right camera, part 0
├── 14625_cam_B_20260510_0108_part001.mp4
└── ...
```

- Any numeric prefix before `cam_A` / `cam_B` is handled automatically.
- Parts are matched by `*cam_A*part*.mp4` / `*cam_B*part*.mp4` and sorted lexicographically.
- Single-part recordings skip the FFmpeg concatenation step.

```bash
./run_folder.sh <input_folder> [results_base]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `input_folder` | — | Folder containing `*cam_A*part*.mp4` and `*cam_B*part*.mp4` |
| `results_base` | `./results` | Parent directory for output; output goes into `<results_base>/<folder_name>/` |

### Output structure

```
results/Montrouge_FC_Chaville_FC_10-05-26/
├── stitched.mp4              ← full H.264 video (720p, CRF 23)
└── stitched/
    ├── master.m3u8           ← HLS playlist
    ├── segment_0000.ts       ← 10-second MPEG-TS segments
    ├── segment_0001.ts
    └── ...
```

Both outputs are always produced:

- **`stitched.mp4`** — for download, review, and archiving.
- **`stitched/`** — HLS segments ready for streaming (HLS.js, video players, CDN upload).

Debug images (`debug_*.jpg`, `dry_run_*.jpg`) land in the same output folder when enabled.

---

## AWS Batch

The Docker image is designed to run as an AWS Batch job. `entrypoint.sh` orchestrates the full pipeline: download → concatenate parts → stitch → upload.

### Input folder convention

Raw videos are stored in S3 as multi-part files, one set per camera:

```
s3://football-raw-videos/match_{uuid}/
├── cam_A_20260331_2039_part000.mp4   ← left camera, part 0
├── cam_A_20260331_2039_part001.mp4   ← left camera, part 1
├── ...
├── cam_B_20260331_2039_part000.mp4   ← right camera, part 0
├── cam_B_20260331_2039_part001.mp4   ← right camera, part 1
└── ...
```

- `cam_A` is always the **left** camera, `cam_B` the **right**.
- Parts are sorted lexicographically before concatenation (`part000 < part001 < ...`).
- Single-part recordings skip the FFmpeg concatenation step entirely.

### Environment variables

Set these via Lambda `containerOverrides` when submitting the Batch job:

| Variable | Description |
|----------|-------------|
| `MATCH_ID` | UUID of the match |
| `INPUT_S3_PREFIX` | S3 folder containing `cam_A_*` and `cam_B_*` parts (e.g. `s3://football-raw-videos/match_{uuid}/`) |
| `MP4_S3_PATH` | Destination for the stitched MP4 (e.g. `s3://football-stitched-videos/match_{uuid}/match_{uuid}_stitched.mp4`) |
| `HLS_S3_PREFIX` | Destination folder for HLS segments (e.g. `s3://vista-stream-segments/match_{uuid}`) |

### Job flow

1. `aws s3 sync` downloads all `cam_A_*.mp4` and `cam_B_*.mp4` parts from `INPUT_S3_PREFIX`.
2. Each camera's parts are concatenated in order using FFmpeg stream copy (no re-encoding).
3. The two concatenated files are passed to `run.sh` → `image-stitching`.
4. The output MP4 is uploaded to `MP4_S3_PATH`.
5. HLS playlist (`master.m3u8`) and segments (`*.ts`) are uploaded to `HLS_S3_PREFIX`.

To build and push the image to ECR:

```bash
./build_and_push_ecr.sh
```

---

## Calibration

The stitcher uses stereo calibration parameters (`K`, `D`, `R`) for accurate undistortion and alignment.

### Converting NPY to YAML

If you have NumPy calibration files from stereo calibration:

```bash
python convert_npy_to_yaml.py <input_folder> <output_folder> [--resolution WxH]
```

**Required NPY files:**
- `cameraMatrix1.npy`, `cameraMatrix2.npy` — intrinsic matrices
- `distCoeffs1.npy`, `distCoeffs2.npy` — distortion coefficients  
- `R.npy` — stereo rotation matrix (cam0 → cam1)
- `T.npy` — stereo translation vector

**Example:**
```bash
python convert_npy_to_yaml.py params_npy params_v1 --resolution 1640x1232
```

### Switching Calibrations

Store multiple calibration sets and switch between them:

```bash
# List available calibrations
./switch_calibration.sh

# Activate a calibration set
./switch_calibration.sh params_v1
```

### Folder Structure

```
stitching/
├── params/              # ← Active calibration (used by stitcher)
│   ├── camchain_0.yaml
│   └── camchain_1.yaml
├── params_v1/           # Calibration set 1
├── params_v2/           # Calibration set 2
└── params_npy/          # Source .npy files
    ├── cameraMatrix1.npy
    ├── R.npy
    └── ...
```

---

## Debug Outputs

When running, the stitcher saves diagnostic images in the output folder:

| File | Description |
|------|-------------|
| `debug_features_0.jpg`, `debug_features_1.jpg` | Detected keypoints per camera |
| `debug_matches_0_1.jpg` | Feature matches between cameras (inliers in green) |
| `debug_overlap_mask_0.jpg`, `debug_overlap_mask_1.jpg` | Overlap region used for feature search |
| `debug_warped_0.jpg`, `debug_warped_1.jpg` | Cylindrically warped images |
| `debug_camera_params.txt` | Final K and R matrices after bundle adjustment |
| `debug_roi_info.txt` | ROI sizes and overlap calculations |
| `dry_run_*.jpg` | Preview of stitched output (dry run mode) |

---

## Troubleshooting

### Few feature matches / poor alignment

- Use `SIFT` instead of `AKAZE` for difficult scenes
- Ensure cameras have 20-40% overlap
- Check `debug_matches_0_1.jpg` — green lines should connect corresponding features

### Bundle adjustment reverts to calibration

If you see `"WARNING: rotation deviated X° from seed"`:
- This means feature matching was unreliable and the stitcher fell back to calibrated rotation
- Usually fine — the calibration is correct, just feature matching was weak
- To improve: ensure better lighting, more texture in overlap zone

### Warping looks wrong at edges

- Verify calibration files match the actual camera setup
- Check that `R.npy` represents cam0→cam1 rotation (not inverted)
- Run dry run and inspect `debug_warped_*.jpg`

### Black borders in output

- Enable LIR cropping: set `use_lir` to `true`
- Or post-process with FFmpeg: `ffmpeg -i output.mp4 -vf "crop=W:H:X:Y" cropped.mp4`

---

## Credits

Based on:

> Du, Chengyao, et al. (2020). *GPU based parallel optimization for real time panoramic video stitching.* Pattern Recognition Letters, 133, 62-69.

Original repository: https://github.com/duchengyao/gpu-based-image-stitching

**Modifications:**
- Two-camera stereo mode with calibration file support
- CLAHE preprocessing + overlap-aware feature masking
- Bundle adjustment with rotation sanity checks
- Multi-band blending with exposure compensation
- Dry run mode and debug visualizations

