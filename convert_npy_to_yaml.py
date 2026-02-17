#!/usr/bin/env python3
"""
Convert stereo calibration data from .npy files to YAML format
for the image stitching pipeline.
"""

import numpy as np
import yaml
import os
import sys
import argparse

def load_npy_params(npy_folder):
    """Load all calibration parameters from .npy files."""
    params = {}
    
    # Load camera matrices (K)
    params['K1'] = np.load(os.path.join(npy_folder, 'cameraMatrix1.npy'))
    params['K2'] = np.load(os.path.join(npy_folder, 'cameraMatrix2.npy'))
    
    # Load distortion coefficients
    params['D1'] = np.load(os.path.join(npy_folder, 'distCoeffs1.npy')).flatten()
    params['D2'] = np.load(os.path.join(npy_folder, 'distCoeffs2.npy')).flatten()
    
    # Load rotation and translation (relative pose between cameras)
    params['R'] = np.load(os.path.join(npy_folder, 'R.npy'))  # Rotation from cam1 to cam2
    params['T'] = np.load(os.path.join(npy_folder, 'T.npy'))  # Translation from cam1 to cam2
    
    # Load rectification rotations (optional, for stereo rectification)
    if os.path.exists(os.path.join(npy_folder, 'R1.npy')):
        params['R1'] = np.load(os.path.join(npy_folder, 'R1.npy'))
        params['R2'] = np.load(os.path.join(npy_folder, 'R2.npy'))
    
    return params

def create_camera_yaml(camera_idx, K, D, R, resolution=(1920, 1080)):
    """Create YAML dictionary for one camera."""
    
    # Extract focal length (average of fx and fy)
    focal = float((K[0, 0] + K[1, 1]) / 2.0)
    
    # Create the YAML structure
    camera_data = {
        'cam_overlaps': [],
        'camera_model': 'pinhole',
        'focal': focal,
        'D': D.tolist(),
        'KMat': {
            '__type__': 'opencv-matrix',
            'rows': 3,
            'cols': 3,
            'dt': 'd',
            'data': K.flatten().tolist()
        },
        'distortion_model': 'radtan',
        'RMat': {
            '__type__': 'opencv-matrix',
            'rows': 3,
            'cols': 3,
            'dt': 'f',
            'data': R.astype(np.float32).flatten().tolist()
        },
        'EYEMat': {
            '__type__': 'opencv-matrix',
            'rows': 3,
            'cols': 3,
            'dt': 'u',
            'data': [1, 0, 0, 0, 1, 0, 0, 0, 1]
        },
        'K': K.flatten().tolist(),
        'R': R.astype(np.float32).flatten().tolist(),
        'resolution': list(resolution),
        'rostopic': f'/cam{camera_idx}/image_raw'
    }
    
    return camera_data

def numpy_representer(dumper, data):
    """Custom YAML representer for numpy arrays."""
    return dumper.represent_list(data.tolist())

def write_yaml_with_opencv_format(filepath, data):
    """Write YAML file with OpenCV-compatible matrix format."""
    
    # Manual YAML writing to match OpenCV format exactly
    with open(filepath, 'w') as f:
        f.write('%YAML:1.0\n')
        f.write(f"cam_overlaps: []\n")
        f.write(f'camera_model: "{data["camera_model"]}"\n')
        f.write(f'focal: {data["focal"]}\n')
        
        # Write distortion coefficients
        f.write(f'D: [{", ".join(map(str, data["D"]))}]\n')
        
        # Write KMat
        f.write('KMat: !!opencv-matrix\n')
        f.write('    rows:3\n')
        f.write('    cols:3\n')
        f.write('    dt:d\n')
        f.write(f'    data:[{", ".join(map(str, data["KMat"]["data"]))}]\n')
        
        # Write distortion model
        f.write(f'distortion_model: {data["distortion_model"]}\n')
        
        # Write RMat
        f.write('RMat: !!opencv-matrix\n')
        f.write('   rows: 3\n')
        f.write('   cols: 3\n')
        f.write('   dt: f\n')
        rmat_data = data["RMat"]["data"]
        f.write('   data: [ ')
        for i in range(0, len(rmat_data), 3):
            if i > 0:
                f.write('       ')
            f.write(f'{rmat_data[i]}, {rmat_data[i+1]}, {rmat_data[i+2]}')
            if i < len(rmat_data) - 3:
                f.write(',\n')
        f.write(' ]\n')
        
        # Write EYEMat
        f.write('EYEMat: !!opencv-matrix\n')
        f.write('    rows:3\n')
        f.write('    cols:3\n')
        f.write('    dt:u\n')
        f.write(f'    data:[{", ".join(map(str, data["EYEMat"]["data"]))}]\n')
        
        # Write K and R as lists
        f.write(f'K: [{", ".join(map(str, data["K"]))}]\n')
        f.write(f'R: [{", ".join(map(str, data["R"]))}]\n')
        
        # Write resolution and rostopic
        f.write(f'resolution: [{data["resolution"][0]}, {data["resolution"][1]}]\n')
        f.write(f'rostopic: "{data["rostopic"]}"\n')

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Convert stereo calibration .npy files to YAML format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert params_npy to params_v1 folder
  python convert_npy_to_yaml.py params_npy params_v1
  
  # Convert with custom resolution
  python convert_npy_to_yaml.py params_npy params_v2 --resolution 1280x720
  
  # Output will be params_v1/camchain_0.yaml and params_v1/camchain_1.yaml
        """
    )
    parser.add_argument('input_folder', 
                        help='Folder containing .npy calibration files (e.g., params_npy)')
    parser.add_argument('output_folder',
                        help='Output folder for YAML files (e.g., params_v1, params_v2)')
    parser.add_argument('--resolution', '-r',
                        help='Image resolution as WIDTHxHEIGHT (default: 1920x1080)',
                        default='1920x1080')
    
    args = parser.parse_args()
    
    # Paths
    npy_folder = args.input_folder
    output_folder = args.output_folder
    
    # Parse resolution
    try:
        width, height = map(int, args.resolution.split('x'))
        resolution = (width, height)
    except:
        print(f"Error: Invalid resolution format '{args.resolution}'. Use WIDTHxHEIGHT (e.g., 1920x1080)")
        sys.exit(1)
    
    # Check if input folder exists
    if not os.path.exists(npy_folder):
        print(f"Error: Input folder '{npy_folder}' not found!")
        sys.exit(1)
    
    # Create output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)
    
    # Load parameters from .npy files
    print(f"Loading calibration data from '{npy_folder}'...")
    try:
        params = load_npy_params(npy_folder)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print(f"\nRequired .npy files in '{npy_folder}':")
        print("  - cameraMatrix1.npy, cameraMatrix2.npy")
        print("  - distCoeffs1.npy, distCoeffs2.npy")
        print("  - R.npy, T.npy")
        sys.exit(1)
    
    print(f"\n{'='*60}")
    print(f"Camera 1 - K matrix:\n{params['K1']}")
    print(f"Camera 1 - Distortion: {params['D1']}")
    print(f"\nCamera 2 - K matrix:\n{params['K2']}")
    print(f"Camera 2 - Distortion: {params['D2']}")
    print(f"\nRelative Rotation R:\n{params['R']}")
    print(f"Relative Translation T: {params['T'].flatten()}")
    print(f"{'='*60}\n")
    
    # Camera 0 (left): use identity rotation (reference frame)
    R0 = np.eye(3, dtype=np.float32)
    
    # Camera 1 (right): use relative rotation R
    R1 = params['R'].astype(np.float32)
    
    print(f"Resolution: {resolution[0]}x{resolution[1]}")
    print(f"Output folder: '{output_folder}'\n")
    
    # Create YAML data for both cameras
    print("Generating YAML files...")
    cam0_data = create_camera_yaml(0, params['K1'], params['D1'], R0, resolution)
    cam1_data = create_camera_yaml(1, params['K2'], params['D2'], R1, resolution)
    
    # Write to YAML files (no suffix in filename, folder distinguishes them)
    output_file_0 = os.path.join(output_folder, 'camchain_0.yaml')
    output_file_1 = os.path.join(output_folder, 'camchain_1.yaml')
    
    write_yaml_with_opencv_format(output_file_0, cam0_data)
    write_yaml_with_opencv_format(output_file_1, cam1_data)
    
    print(f"\n{'='*60}")
    print(f"✓ Created: {output_file_0}")
    print(f"✓ Created: {output_file_1}")
    print(f"{'='*60}")
    print("\nYAML files generated successfully!")
    print("\nNote: Camera 0 uses identity rotation (reference frame)")
    print("      Camera 1 uses relative rotation from stereo calibration")
    print(f"\nTo use these files, run:")
    print(f"  ./switch_calibration.sh {output_folder}")


if __name__ == '__main__':
    main()
