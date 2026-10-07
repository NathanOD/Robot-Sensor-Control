import ast
import yaml
import numpy as np
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from core.transform import rotvec_to_rotation_matrix, rotation_matrix_to_rotvec, tcp_entry_to_list, get_base_to_table_matrix

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

class RobotController:
    def __init__(self, config_dir="config"):
        self.config_dir = config_dir
        
        network_conf = load_yaml(f"{config_dir}/network_config.yaml")
        self.ip = network_conf.get('robot')
        
        self.tcp_defs = load_yaml(f"{config_dir}/tcp_config.yaml")
        self.table_conf = load_yaml(f"{config_dir}/table_config.yaml")
        
        print(f"Connecting to robot at {self.ip}...")
        self.rtde_r = RTDEReceiveInterface(self.ip)
        self.rtde_c = RTDEControlInterface(self.ip)
        
        self.current_tcp_name = None

    def set_active_tcp(self, tcp_name):
        """
        Set the active TCP by name. The TCP definitions are loaded from tcp_config.yaml.

        Args:
            tcp_name (str): The name of the TCP to set as active.
        """
        if tcp_name not in self.tcp_defs:
            raise ValueError(f"TCP '{tcp_name}' not found in tcp.yaml. Available: {list(self.tcp_defs.keys())}")
        selected_tcp = tcp_entry_to_list(self.tcp_defs[tcp_name])
        self.rtde_c.setTcp(selected_tcp)
        self.current_tcp_name = tcp_name
        print(f"TCP set to '{tcp_name}': {selected_tcp}")

    def go_home(self, speed=0.5, acc=0.5):
        """
        Move the robot to the home position, defined in home_config.yaml.

        Args:
            speed (float): The speed at which to move to the home position.
            acc (float): The acceleration at which to move to the home position.
        
        Returns:
            None
        """
        home_conf = load_yaml(f"{self.config_dir}/home_config.yaml")
        if 'home' not in home_conf or 'target_q' not in home_conf['home']:
            raise ValueError("home_config.yaml must contain 'home' key with 'target_q' array.")

        raw_target_q = home_conf['home']['target_q']

        # Support both YAML list styles and legacy stringified lists.
        if isinstance(raw_target_q, str):
            try:
                raw_target_q = ast.literal_eval(raw_target_q)
            except Exception as e:
                raise ValueError(f"Invalid 'target_q' string format in home_config.yaml: {e}")

        if not isinstance(raw_target_q, (list, tuple)):
            raise ValueError("home_config.yaml 'home.target_q' must be a list of 6 joint values.")

        if len(raw_target_q) != 6:
            raise ValueError(f"home_config.yaml 'home.target_q' must contain 6 values, got {len(raw_target_q)}.")

        home_q = [float(q) for q in raw_target_q]
        print(f"Moving to home joints: {home_q}")
        self.rtde_c.moveJ(home_q, speed, acc)
        print("✓ Robot moved successfully to home position.")

    def go_to_table_center(self, z, rx=0.0, ry=0.0, rz=0.0, speed=0.2, acc=0.2):
        """
        Move the robot to the center of the table at a specified height.
        
        Args:
            z (float): The target height (Z coordinate) in the table frame.
            rx (float): Rotation around the X-axis in radians.
            ry (float): Rotation around the Y-axis in radians.
            rz (float): Rotation around the Z-axis in radians.
            speed (float): The speed at which to move to the target pose.
            acc (float): The acceleration at which to move to the target pose.
        """
        cx = self.table_conf['width'] / 2.0
        cy = self.table_conf['height'] / 2.0
        print(f"Moving to table center (x={cx:.3f}, y={cy:.3f}) at height z={z:.3f}...")
        self.move_to_pose(x=cx, y=cy, z=z, rx=rx, ry=ry, rz=rz, frame="table", speed=speed, acc=acc)

    def move_to_pose(self, x, y, z, rx=0.0, ry=0.0, rz=0.0, frame="table", speed=0.2, acc=0.2):
        """
        Move the robot to a specified pose in either the 'table' or 'base' frame.
        
        Args:
            x (float): The X coordinate of the target pose.
            y (float): The Y coordinate of the target pose.
            z (float): The Z coordinate of the target pose.
            rx (float): The rotation around the X-axis (in radians) for the target pose.
            ry (float): The rotation around the Y-axis (in radians) for the target pose.
            rz (float): The rotation around the Z-axis (in radians) for the target pose.
            frame (str): The reference frame for the target pose ('table' or 'base').
            speed (float): The speed at which to move to the target pose.
            acc (float): The acceleration at which to move to the target pose.
        
        Returns:
            None
        """
        if frame == 'table':
            base_to_table = get_base_to_table_matrix(self.table_conf)

            target_in_table = np.eye(4)
            target_in_table[0:3, 0:3] = rotvec_to_rotation_matrix([rx, ry, rz])
            target_in_table[0:3, 3] = [x, y, z]

            target_in_base = base_to_table @ target_in_table
            base_pos = target_in_base[0:3, 3]
            base_rotvec = rotation_matrix_to_rotvec(target_in_base[0:3, 0:3])

            target_pose = [
                float(base_pos[0]), float(base_pos[1]), float(base_pos[2]),
                float(base_rotvec[0]), float(base_rotvec[1]), float(base_rotvec[2])
            ]
        else:
            target_pose = [x, y, z, rx, ry, rz]

        print(f"Moving to target pose in {frame} frame: x={x:.3f}, y={y:.3f}, z={z:.3f}...")
        self.rtde_c.moveL(target_pose, speed, acc)
        
    def get_actual_q(self):
        # Added reconnection fix (see: https://gitlab.com/sdurobotics/ur_rtde/-/work_items/102#note_862758754)
        if self.rtde_r.isConnected() == False:
            self.rtde_r.reconnect()
        return self.rtde_r.getActualQ()
        
    def get_actual_tcp_pose(self):
        # Added reconnection fix (see: https://gitlab.com/sdurobotics/ur_rtde/-/work_items/102#note_862758754)
        if self.rtde_r.isConnected() == False:
            self.rtde_r.reconnect()
        return self.rtde_r.getActualTCPPose()
