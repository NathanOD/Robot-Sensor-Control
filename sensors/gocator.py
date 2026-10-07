import os
import yaml
import numpy as np
from PIL import Image

# Import the CFFI wrapper for Gocator SDK
try:
    from ._gocator_wrapper import ffi, lib #type: ignore
except ImportError as e:
    print(f"Warning: _gocator_wrapper not found. Exception: {e}")
    ffi, lib = None, None

def load_yaml(file_path):
    """
    Load a YAML file and return its contents as a dictionary.

    Args:
        file_path (str): The path to the YAML file to load.

    Returns:
        dict: The contents of the YAML file as a dictionary.
    """
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        raise ValueError(f"Failed to read YAML file {file_path}: {e}")

class GocatorCamera:
    def __init__(self, config_dir="config"):
        network_conf = load_yaml(f"{config_dir}/network_config.yaml")
        self.ip = network_conf.get('gocator')
        print(f"Connecting to Gocator at {self.ip}...")
        
        if lib and ffi:
            status = lib.gocator_connect(self.ip.encode('utf-8'))
            if status != 1:  # kOK is 1 in Gocator SDK
                raise ConnectionError(f"Failed to connect to Gocator: {status}")

    def take_scan(self):
        """
         Trigger a scan and retrieve point cloud data from the Gocator.

         Returns:
            Dictionary with scan status and point cloud data.
         """
        print(f"Gocator {self.ip} : Scanning profile...")
        if not lib or not ffi:
            return {"status": "error", "data": []}
            
        lib.gocator_start()
        lib.gocator_trigger()
        
        max_points = 5000000  # 5M points max
        out_buffer = ffi.new(f"double[{max_points * 3}]")
        points_received = ffi.new("int*")
        
        status = lib.gocator_receive_data(out_buffer, max_points, points_received)
        lib.gocator_stop()
        
        if status == 1 and points_received[0] > 0:
            data_np = np.frombuffer(ffi.buffer(out_buffer, points_received[0] * 3 * 8), dtype=np.float64)
            points_xyz = data_np.reshape(-1, 3)
            
            # Gocator G3 uses left-handed coordinate system. Flip X to convert to right-handed.
            points_xyz[:, 0] = -points_xyz[:, 0]
            
            print(f"Acquisition success: {points_received[0]} points received.")
            return {"status": "success", "data": points_xyz}
        else:
            return {"status": "error", "data": []}
    
    def take_intensity_image(self):
        """
         Trigger a scan and retrieve intensity image from the Gocator.

         Returns:
            Dictionary with status, width, height, and image data.
         """
        print(f"Gocator {self.ip} : Acquiring intensity image...")
        if not lib or not ffi:
            return {"status": "error", "width": 0, "height": 0, "data": None}
        
        lib.gocator_start()
        lib.gocator_trigger()
        
        max_size = 5000000  # Max resolution (1668x2568 = 4.28M pixels)
        out_buffer = ffi.new(f"unsigned char[{max_size}]")
        width = ffi.new("int*")
        height = ffi.new("int*")
        size_received = ffi.new("int*")
        
        status = lib.gocator_receive_intensity_image(out_buffer, max_size, width, height, size_received)
        lib.gocator_stop()
            
        if status == 1 and size_received[0] > 0:
            data_np = np.frombuffer(ffi.buffer(out_buffer, size_received[0]), dtype=np.uint8)
            image_data = data_np.reshape((height[0], width[0]))
            
            print(f"Acquisition success: {width[0]} x {height[0]} pixels.")
            return {"status": "success", "width": width[0], "height": height[0], "data": image_data}
        else:
            return {"status": "error", "width": 0, "height": 0, "data": []}

    def save_scan(self, points, pose_num=0, save_dir=None, file_prefix="pose_", format="txt"):
        """
         Save the point cloud data to a text or PLY file.
         
         Args:
            points (Nx3 numpy array): Point cloud data
            pose_num (int): Pose number for filename
            save_dir (str): Directory to save the scan
            file_prefix (str): Prefix for the filename
            format (str): Output format ("txt" or "ply")

        Returns:
            None
         """
        os.makedirs(save_dir, exist_ok=True)  # Create directory if it doesn't exist
        
        # Filename with pose number, zero-padded to 2 digits
        filename = os.path.join(save_dir, f"{file_prefix}{pose_num:02d}.{format.lower()}")
        
        if format.lower() == "ply":
            with open(filename, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
                np.savetxt(f, points, delimiter=" ", fmt="%.4f")
        else:
            np.savetxt(filename, points, delimiter=" ", comments="", fmt="%.2f")

    def save_intensity_image(self, image_data, pose_num=0, save_dir=None, file_prefix="pose_"):
        """
         Save the intensity image to a BMP file in the data_gocator/images directory.
         
         Args:
            image_data (HxW numpy array): Intensity image data (uint8)
            pose_num (int): Pose number for filename
            save_dir (str): Directory to save the image
            file_prefix (str): Prefix for the filename

        Returns:
            None
         """
        os.makedirs(save_dir, exist_ok=True) # Create directory if it doesn't exist
        
        # Filename with pose number, zero-padded to 2 digits
        filename = os.path.join(save_dir, f"{file_prefix}{pose_num:02d}.bmp")
        
        # Save the image using PIL
        img = Image.fromarray(image_data, mode='L')  # L = grayscale
        img.save(filename)

    def __del__(self):
        if lib:
            try:
                lib.gocator_disconnect()
            except:
                pass
