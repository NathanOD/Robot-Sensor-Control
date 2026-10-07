import json
import yaml
import shutil
import subprocess
from pathlib import Path


class ZedXCamera:
    def __init__(self, config_dir="config"):
        self.config_dir = config_dir
        self.ssh_config = self._load_ssh_config()
        print(f"Connecting to ZED-X at {self.ssh_config['host']}...")
        self._check_ssh_connection()
        self._captures_dir_initialized = False

    def _check_ssh_connection(self):
        """
        Validate SSH access to the Jetson and fail fast if unreachable.
        """
        try:
            self._run_ssh_command("echo connected")
        except subprocess.CalledProcessError as e:
            error_msg = (e.stderr or "").strip()
            if error_msg:
                raise ConnectionError(f"Failed to connect to Jetson via SSH: {error_msg}") from e
            raise ConnectionError("Failed to connect to Jetson via SSH") from e

    def _load_ssh_config(self):
        """
        Load SSH configuration from jetson_config.yaml.
        """
        config_path = Path(self.config_dir) / "jetson_config.yaml"
        with open(config_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        
        jetson_config = config.get("jetson", {})
        jetson_config.setdefault("port", 22)
        return jetson_config

    def _ensure_sshpass_available(self):
        """
        Verify that sshpass is installed.
        """
        if shutil.which("sshpass") is None:
            raise RuntimeError("sshpass is required for SSH/SCP operations")

    def _run_ssh_command(self, command):
        """
        Execute a command on the Jetson via SSH.

        Args:
            command (str): The command to execute on the Jetson

        Returns:
            str: The standard output from the command execution
        """
        self._ensure_sshpass_available()
        remote_target = f'{self.ssh_config["username"]}@{self.ssh_config["host"]}'
        ssh_command = [
            "sshpass",
            "-p",
            self.ssh_config["password"],
            "ssh",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-p",
            str(self.ssh_config["port"]),
            remote_target,
            command,
        ]
        result = subprocess.run(ssh_command, check=True, text=True, capture_output=True)
        return result.stdout.strip()

    def _download_file(self, remote_path, local_path):
        """
        Download a file from the Jetson via SCP.

        Args:
            remote_path (str): The path to the file on the Jetson
            local_path (str or Path): The local path where the file should be saved
        
        Returns:
            None
        """
        self._ensure_sshpass_available()
        remote_target = f'{self.ssh_config["username"]}@{self.ssh_config["host"]}:{remote_path}'
        scp_command = [
            "sshpass",
            "-p",
            self.ssh_config["password"],
            "scp",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-P",
            str(self.ssh_config["port"]),
            remote_target,
            str(local_path),
        ]
        subprocess.run(scp_command, check=True)

    def _download_directory(self, remote_dir, local_dir):
        """
        Download a directory from the Jetson via SCP

        Args:
            remote_dir (str): The path to the directory on the Jetson
            local_dir (str or Path): The local path where the directory should be saved
        
        Returns:
            None
        """
        self._ensure_sshpass_available()
        
        # Replace $HOME with the remote absolute home path
        if "$HOME" in remote_dir:
            try:
                remote_home = self._run_ssh_command("echo $HOME")
                remote_dir = remote_dir.replace("$HOME", remote_home)
            except subprocess.CalledProcessError:
                pass
        
        # Create local directory if it doesn't exist
        Path(local_dir).mkdir(parents=True, exist_ok=True)
        
        remote_target = f'{self.ssh_config["username"]}@{self.ssh_config["host"]}:{remote_dir}/*'
        scp_command = [
            "sshpass",
            "-p",
            self.ssh_config["password"],
            "scp",
            "-r",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-P",
            str(self.ssh_config["port"]),
            remote_target,
            str(local_dir),
        ]
        subprocess.run(scp_command, check=True)

    def acquire_intrinsics(self, output_dir="data_zedx"):
        """
        Acquire camera intrinsics from ZED-X on Jetson and save locally.
        
        Args:
            output_dir (str): Local directory where intrinsics will be saved
            
        Returns:
            Dictionary containing intrinsics (fx, fy, cx, cy)
        """
        print("Acquiring ZED-X camera intrinsics...")
        
        # Create output directory
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        
        # Get remote home directory
        remote_home = self._run_ssh_command("echo $HOME")
        remote_tmp_dir = f"{remote_home}/zedx_intrinsics_tmp"
        
        # Create remote temporary directory
        self._run_ssh_command(f"mkdir -p {remote_tmp_dir}")
        
        try:
            # Run zedx_acquisition.py on Jetson to capture intrinsics only
            print("  Capturing intrinsics from ZED-X on Jetson...")
            self._run_ssh_command(
                f"python zedx_acquisition.py "
                f"--output-dir {remote_tmp_dir} --save-intrinsics-only"
            )
            
            # Download the intrinsics file
            print("  Downloading intrinsics file...")
            remote_intrinsics = f"{remote_tmp_dir}/intrinsics_zedx.json"
            local_intrinsics = Path(output_dir) / "intrinsics_zedx.json"
            self._download_file(remote_intrinsics, local_intrinsics)
            
            # Load and return intrinsics
            with open(local_intrinsics, "r", encoding="utf-8") as f:
                intrinsics = json.load(f)
            
            print(f"✓ Intrinsics acquired successfully: {intrinsics}")
            return intrinsics
            
        finally:
            # Clean up remote temporary directory
            self._run_ssh_command(f"rm -rf {remote_tmp_dir}")

    def capture_rgbd(self, max_depth=1.5):
        """
        Capture RGB + depth image from ZED-X on Jetson.
        Images are accumulated on Jetson. Use download_captures() to download all at the end.
        
        Args:
            max_depth (float): Maximum depth distance in meters
            
        Returns:
            Dictionary with capture status
        """
        print("ZED-X : Grabbing image...")
        
        try:
            # Ensure a clean remote capture directory once at the beginning of a run.
            if not self._captures_dir_initialized:
                self._run_ssh_command("rm -rf $HOME/zedx_captures && mkdir -p $HOME/zedx_captures")
                self._captures_dir_initialized = True
            else:
                self._run_ssh_command("mkdir -p $HOME/zedx_captures")
            
            # Run zedx_acquisition.py on Jetson to capture one image
            self._run_ssh_command(
                f"python zedx_acquisition.py "
                f"--output-dir $HOME/zedx_captures --max-depth {max_depth}"
            )
            return {"status": "success"}
        except subprocess.CalledProcessError as e:
            print(f"Error capturing image: {e}")
            return {"status": "failed"}

    def download_captures(self, output_dir="data_zedx"):
        """
        Download all captured images from Jetson to local directory.
        Call this method after all grab_image() calls are done.
        
        Args:
            output_dir (str): Local directory where all images will be saved
        
        Returns:
            Dictionary with download status and output directory paths
        """
        image_folder_name = "captures"
        print(f"\nDownloading all captured images to {output_dir}/{image_folder_name}...")
        
        # Create output directory
        images_dir = Path(output_dir) / f"{image_folder_name}"
        images_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            # Download the entire directory
            self._download_directory("$HOME/zedx_captures", images_dir)
            print(f"✓ All images downloaded successfully to {images_dir}")
            return {"status": "success", "output_dir": str(images_dir)}
        except subprocess.CalledProcessError as e:
            print(f"Error downloading images: {e}")
            return {"status": "failed"}
        finally:
            # Clean up remote directory
            self._run_ssh_command("rm -rf $HOME/zedx_captures")
            self._captures_dir_initialized = False
