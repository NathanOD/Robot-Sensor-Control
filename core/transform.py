import json
import yaml
import numpy as np
from pathlib import Path

def dh(a, d, alpha, theta):
    """
    Compute the Denavit-Hartenberg transformation matrix.
    
    Args:
        a (float): Link length
        d (float): Link offset
        alpha (float): Link twist
        theta (float): Joint angle
    
    Returns:
        numpy.ndarray: The 4x4 transformation matrix
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])

def rotvec_to_rotation_matrix(rotvec):
    """
    Convert a rotation vector to a rotation matrix using Rodrigues' formula.
    
    Args:
        rotvec (numpy.ndarray): A 3D rotation vector (rx, ry, rz)
    
    Returns:
        numpy.ndarray: The 3x3 rotation matrix
    """
    angle = np.linalg.norm(rotvec)
    if np.isclose(angle, 0.0):
        return np.eye(3)
    axis = rotvec / angle
    K = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0]
    ])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)

def rotation_matrix_to_rotvec(rotation_matrix):
    """
    Convert a rotation matrix to a rotation vector using the logarithm map.
    
    Args:
        rotation_matrix (numpy.ndarray): The 3x3 rotation matrix
    
    Returns:
        numpy.ndarray: The 3D rotation vector (rx, ry, rz)
    """
    trace_value = np.trace(rotation_matrix)
    cos_angle = (trace_value - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)

    if np.isclose(angle, 0.0):
        return np.zeros(3)

    if np.isclose(angle, np.pi):
        M = (rotation_matrix + np.eye(3)) / 2.0
        diag = np.diag(M)
        i = np.argmax(diag)
        v = np.zeros(3)
        v[i] = np.sqrt(max(0.0, diag[i]))
        for j in range(3):
            if i != j:
                v[j] = M[i, j] / (v[i] + 1e-12)
        norm = np.linalg.norm(v)
        if np.isclose(norm, 0.0):
            return np.zeros(3)
        return v / norm * angle

    skew = (rotation_matrix - rotation_matrix.T) / (2.0 * np.sin(angle))
    axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    return axis * angle

def tcp_entry_to_list(entry):
    """
    Convert a TCP entry to a list of floats.
    
    Args:
        entry (dict): A dictionary containing TCP values
    
    Returns:
        list: A list of floats representing the TCP values
    """
    return [float(entry.get(k, 0.0)) for k in ("x", "y", "z", "rx", "ry", "rz")]

def get_base_to_table_matrix(table_conf):
    """
    Get the 4x4 transformation matrix from the table frame to the base frame.

    Args:
        table_conf (dict): The table configuration dictionary containing delta_x, delta_y, delta_z
            and optional rot_x, rot_y, rot_z (rotation vector in radians). Defaults to 180° around
            Z when rotation fields are absent, which inverts X and Y axes.

    Returns:
        numpy.ndarray: The 4x4 transformation matrix (H_base_table)
    """
    rot_x = table_conf.get('rot_x', 0.0)
    rot_y = table_conf.get('rot_y', 0.0)
    rot_z = table_conf.get('rot_z', np.pi)
    base_to_table = np.eye(4)
    base_to_table[0:3, 0:3] = rotvec_to_rotation_matrix([rot_x, rot_y, rot_z])
    base_to_table[0:3, 3] = [table_conf['delta_x'], table_conf['delta_y'], table_conf['delta_z']]
    return base_to_table

def transform_point(matrix, point):
    """
    Transform a 3D point using a 4x4 transformation matrix.
    
    Args:
        matrix (numpy.ndarray): 4x4 transformation matrix.
        point (list or numpy.ndarray): 3D point [x, y, z].
        
    Returns:
        numpy.ndarray: Transformed 3D point [x, y, z].
    """
    if point is None:
        return None
    p_homogenous = np.array([point[0], point[1], point[2], 1.0])
    p_transformed = matrix @ p_homogenous
    return p_transformed[:3]

def transform_vector(matrix, vector):
    """
    Transform a 3D vector using a 4x4 transformation matrix.
    Translation is ignored for vectors.
    
    Args:
        matrix (numpy.ndarray): 4x4 transformation matrix.
        vector (list or numpy.ndarray): 3D vector [x, y, z].
        
    Returns:
        numpy.ndarray: Transformed 3D vector [x, y, z].
    """
    if vector is None:
        return None
    v_homogenous = np.array([vector[0], vector[1], vector[2], 0.0])
    v_transformed = matrix @ v_homogenous
    return v_transformed[:3]

def transform_detection_results(results, transform_matrix):
    """
    Transform the joint detection results to a different coordinate frame.
    
    Args:
        results (list): List of joint detection result dictionaries.
        transform_matrix (numpy.ndarray): 4x4 transformation matrix to apply.
        
    Returns:
        list: A new list of transformed joint detection results.
    """
    transformed_results = []
    for item in results:
        new_item = {
            'intersection_id': item['intersection_id'],
            'start_point': transform_point(transform_matrix, item['start_point']).tolist() if item['start_point'] else None,
            'end_point': transform_point(transform_matrix, item['end_point']).tolist() if item['end_point'] else None,
            'coordinate_frame': {
                'origin': transform_point(transform_matrix, item['coordinate_frame']['origin']).tolist(),
                'x_axis': transform_vector(transform_matrix, item['coordinate_frame']['x_axis']).tolist(),
                'y_axis': transform_vector(transform_matrix, item['coordinate_frame']['y_axis']).tolist(),
                'z_axis': transform_vector(transform_matrix, item['coordinate_frame']['z_axis']).tolist()
            }
        }
        transformed_results.append(new_item)
    return transformed_results

def convert_points_base_to_table(points, table_conf):
    """
    Convert a list or array of 3D points from the robot base frame to the table frame.
    
    Args:
        points (list or numpy.ndarray): List of 3D points [x, y, z] in base frame.
        table_conf (dict): The table configuration dictionary.
        
    Returns:
        numpy.ndarray: Array of transformed 3D points in the table frame.
    """
    base_to_table = get_base_to_table_matrix(table_conf)
    table_to_base = np.linalg.inv(base_to_table)
    
    transformed_points = []
    for point in points:
        transformed_points.append(transform_point(table_to_base, point))
    return np.asarray(transformed_points)
