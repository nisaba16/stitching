# Fast panorama stitching method using UMat.

Code is based on the following work, but it is heavily modified version of it. The modifications include:

- Use always only two input videos
- Use LIR
- Change the logic of video reading to get rid of semaphores
- Add a dry run mode
- Add a mode to output the intermediate images
- Fix issues related to infinite stitching loop in the original code
- And some other minor changes


Paper: 

> Du, Chengyao, et al. (2020). GPU based parallel optimization for real time panoramic video stitching. Pattern Recognition Letters, 133, 62-69.
 

Repository:

> https://github.com/duchengyao/gpu-based-image-stitching


## How to run

### Option 1: Using Docker (Recommended - Simplest)

Build the Docker image:
```bash
docker build -t image-stitching:latest .
```

Run with just the two video files (FPS is auto-detected):
```bash
docker run -v /path/to/videos:/videos -v /path/to/output:/output image-stitching:latest /videos/left.mp4 /videos/right.mp4
```

With custom output folder/name:
```bash
docker run -v /path/to/videos:/videos -v /path/to/output:/output image-stitching:latest /videos/left.mp4 /videos/right.mp4 /output my_stitched_video
```

If using calibration files, mount the `params` folder:
```bash
docker run -v $(pwd)/params:/app/params \
           -v /path/to/videos:/videos \
           -v /path/to/output:/output \
           image-stitching:latest /videos/left.mp4 /videos/right.mp4
```

The wrapper script automatically:
- ✅ Detects FPS from the first video
- ✅ Creates the output folder if needed
- ✅ Detects and uses calibration files if they exist
- ✅ Uses sensible defaults for other parameters

### Option 2: Local Build

```bash
$ mkdir build && cd build
$ cmake .. && make
$ ./image-stitching <output_folder> <file_name> <fps> <dry_run> <use_lir> <use_calibration> <use_sift> <video_file1> <video_file2>
```

Parameters:
- `output_folder`: Directory to save the stitched video
- `file_name`: Output file name (without extension)
- `fps`: Frames per second for output video
- `dry_run`: `true` to process without saving output, `false` to save
- `use_lir`: `true` to use Largest Interior Rectangle cropping, `false` otherwise
- `use_calibration`: `true` to use YAML calibration files from `params/`, `false` for on-the-fly calibration
- `use_sift`: `true` for SIFT feature detector (accurate), `false` for ORB (fast)
- `video_file1`, `video_file2`: Paths to the two input video files

### Examples

Using calibration files with SIFT (requires `params/camchain_*.yaml`):
```bash
./image-stitching ../results stitched_video 30 false false true true /path/to/left.mp4 /path/to/right.mp4
```

On-the-fly calibration with ORB (no YAML files needed):
```bash
./image-stitching ../results stitched_video 30 false false false false /path/to/left.mp4 /path/to/right.mp4
```

## Calibration Management

The project supports multiple calibration parameter sets stored in separate folders that can be easily switched between.

### Calibration Workflow

1. **Organize calibration data**: Store stereo calibration `.npy` files in a folder (e.g., `params_npy/`)
   - Required files: `cameraMatrix1.npy`, `cameraMatrix2.npy`, `distCoeffs1.npy`, `distCoeffs2.npy`, `R.npy`, `T.npy`

2. **Convert to YAML format**: Run the conversion script specifying input and output folders
   ```bash
   # Convert params_npy to params_v1 folder
   python convert_npy_to_yaml.py params_npy params_v1
   # Creates: params_v1/camchain_0.yaml, params_v1/camchain_1.yaml
   
   # Convert with custom resolution
   python convert_npy_to_yaml.py my_calibration params_v2 --resolution 1920x1080
   # Creates: params_v2/camchain_0.yaml, params_v2/camchain_1.yaml
   ```

3. **Switch between calibrations**: Use the switch script to activate a calibration folder
   ```bash
   # List available calibrations
   ./switch_calibration.sh
   
   # Switch to a specific calibration folder (e.g., params_v1, params_v2, params_old)
   ./switch_calibration.sh params_v1
   ```
   This copies files from the specified folder to `params/` (active calibration used by stitching)

4. **Run stitching**: The active calibration in `params/` will be used
   ```bash
   ./image-stitching ../results output 30 false false true true ../datasets/match_20260215_1520_cam0_0001.mp4 ../datasets/match_20260215_1520_cam1_0001.mp4
   ```

### Calibration Folder Structure

Each calibration folder contains:
- `camchain_0.yaml` - Left camera parameters (K matrix, distortion, rotation)
- `camchain_1.yaml` - Right camera parameters

Example structure:
```
stitching/
├── params/              # Active calibration (used by stitching algorithm)
│   ├── camchain_0.yaml
│   └── camchain_1.yaml
├── params_v1/           # Calibration version 1
│   ├── camchain_0.yaml
│   └── camchain_1.yaml
├── params_v2/           # Calibration version 2
│   ├── camchain_0.yaml
│   └── camchain_1.yaml
└── params_npy/          # Source .npy files
    ├── cameraMatrix1.npy
    ├── cameraMatrix2.npy
    └── ...
```

### Example Workflow
```bash
# Convert multiple calibration sets
python convert_npy_to_yaml.py params_npy_v1 params_v1
python convert_npy_to_yaml.py params_npy_v2 params_v2

# Test with v1 calibration
./switch_calibration.sh params_v1
./image-stitching ../results test_v1 30 false false true true left.mp4 right.mp4

# Test with v2 calibration
./switch_calibration.sh params_v2
./image-stitching ../results test_v2 30 false false true true left.mp4 right.mp4

# Compare results and keep the best one
```

