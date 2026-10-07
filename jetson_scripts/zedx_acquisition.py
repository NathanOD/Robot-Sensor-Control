import sys
import os
import json
import argparse
import cv2
import numpy as np
import pyzed.sl as sl

MIN_DEPTH = 0.1

def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture RGB + depth from ZEDX"
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Output directory for captures",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=1.5,
        help="Maximum depth distance in meters (must be > {MIN_DEPTH}m)",
    )
    parser.add_argument(
        "--save-intrinsics-only",
        action="store_true",
        help="Save intrinsics only and exit",
    )
    return parser.parse_args()

args = parse_args()
if args.max_depth <= MIN_DEPTH:
    print(f"[ERROR] max-depth must be greater than {MIN_DEPTH}m")
    sys.exit(1)


def save_left_intrinsics(camera, save_dir):
    intrinsics_path = os.path.join(save_dir, "intrinsics_zedx.json")
    try:
        cam_info = camera.get_camera_information()
        left_cam = cam_info.camera_configuration.calibration_parameters.left_cam
        intrinsics = {
            "fx": float(left_cam.fx),
            "fy": float(left_cam.fy),
            "cx": float(left_cam.cx),
            "cy": float(left_cam.cy),
        }
        with open(intrinsics_path, "w", encoding="utf-8") as f:
            json.dump(intrinsics, f, indent=2)
    except Exception as e:
        print(f"[ERROR] Could not save camera intrinsics: {e}")


def _next_available_index(output_dir, prefix="image", ext=".png"):
    idx = 0
    while True:
        candidate = os.path.join(output_dir, f"{prefix}_{idx:02d}{ext}")
        if not os.path.exists(candidate):
            return idx
        idx += 1

# Initialize ZEDX camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD1200
init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
init_params.coordinate_units = sl.UNIT.METER
init_params.depth_maximum_distance = args.max_depth
init_params.depth_minimum_distance = MIN_DEPTH

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"[ERROR] Camera open failed with error: {err}")
    exit(-1)

# If the user only wants to save intrinsics, do that and exit.
if args.save_intrinsics_only:
    save_left_intrinsics(zed, args.output_dir)
    zed.close()
    sys.exit(0)


image = sl.Mat()
depth = sl.Mat()
#confidence_map = sl.Mat()

if zed.grab() != sl.ERROR_CODE.SUCCESS:
    print("Grab failed")
else:
    zed.retrieve_image(image, sl.VIEW.LEFT)
    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
    #zed.retrieve_measure(confidence_map, sl.MEASURE.CONFIDENCE)

    img_cv = image.get_data()
    depth_cv = depth.get_data().astype(np.float32)
    #confidence_cv = confidence_map.get_data()

    depth_cv = np.nan_to_num(depth_cv, nan=0.0, posinf=0.0, neginf=0.0)
    depth_clipped = np.clip(
        depth_cv, init_params.depth_minimum_distance, init_params.depth_maximum_distance
    ).astype(np.float32)

    idx = _next_available_index(args.output_dir, prefix="image", ext=".png")
    image_filename = os.path.join(args.output_dir, f"image_{idx:02d}.png")
    depth_filename = os.path.join(args.output_dir, f"depth_{idx:02d}.png")

    cv2.imwrite(image_filename, img_cv)
    depth_mm = (depth_clipped * 1000.0).astype(np.uint16)
    cv2.imwrite(depth_filename, depth_mm)

zed.close()