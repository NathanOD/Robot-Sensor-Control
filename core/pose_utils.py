import os
import numpy as np
from core.transform import dh

def get_flange_pose_matrix(q, use_calibration=True):
    """
    Compute the transformation matrix of the robot flange based on joint angles q.
    Applies calibration corrections to the DH parameters unless use_calibration is False.

    Args:
        q (list or array): List or array of 6 joint angles in radians.
        use_calibration (bool): If False, use the nominal UR10e DH parameters
            without the calibration corrections (matches the simulation/RoboDK model).

    Returns:
        4x4 numpy array representing the homogeneous transformation matrix of the flange.
    """
    # DH params UR10e with calibration corrections
    if use_calibration:
        delta_theta = np.array([2.17811880620297216e-07, -0.0578620100776364027, 6.66488832226374139, -0.32384485955561193, 2.30679149890861135e-06, 1.63909438785636574e-07])
        delta_a = np.array([5.95015304749575585e-05, 0.00147174654833370777, 0.0298904544420336427, -2.34208801560553749e-05, 7.29756622126643141e-05, 0])
        delta_d = np.array([2.96502309055646229e-05, -29.746588259262488, 91.9740591982876197, -62.2275001226891149, -3.75969430665146209e-05, -0.00088907773373303467])
        delta_alpha = np.array([-0.000289506217843849001, 0.0011902058273660992, 0.00292177471482431717, -0.00199968903513791929, -0.000994090465941122048, 0])
    else:
        delta_theta = np.zeros(6)
        delta_a = np.zeros(6)
        delta_d = np.zeros(6)
        delta_alpha = np.zeros(6)

    a = np.array([0, -0.6127, -0.57155, 0, 0, 0]) + delta_a
    d = np.array([0.1807, 0, 0, 0.17415, 0.11985, 0.11655]) + delta_d
    alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]) + delta_alpha

    q = np.array(q) + delta_theta

    T = np.eye(4)
    for i in range(6):
        T = T @ dh(a[i], d[i], alpha[i], q[i])
        
    return T

def save_pose_matrix(T, pose_num=0, save_dir=None, file_prefix="pcd_"):
    """
    Save the homogeneous transformation matrix T to a text file in the specified directory.
    The matrix is saved in millimeters with a specific RoboDK format.
    
    Args:
        T (4x4 numpy array): 4x4 numpy array representing the homogeneous transformation matrix.
        pose_num (int): Integer pose number for filename.
        save_dir (str): Directory where the file will be saved. default is project root.
        file_prefix (str): Prefix for the filename, default is "pcd_".
    
    Returns:
        None
    """
    if save_dir is None:
        # Default to the data_gocator/poses folder at the project root, next to the saved scans
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        save_dir = os.path.join(root_dir, "data_gocator", "poses")
        
    os.makedirs(save_dir, exist_ok=True)
    
    filename = os.path.join(save_dir, f"{file_prefix}{pose_num:02d}.txt")
    
    T_mm = T.copy()
    T_mm[0:3, 3] = T_mm[0:3, 3] * 1000  # Convert translations from m to mm

    with open(filename, 'w') as f:
        for i in range(4):
            if i == 0:
                row = "[ "
            else:
                row = "  "
            for j in range(4):
                row += f"{T_mm[i, j]:12.6f},"
            row = row.rstrip(',')
            if i < 3:
                row += " ;\n"
            else:
                row += " ];\n"
            f.write(row)
    print(f"Matrix saved to {filename}")
