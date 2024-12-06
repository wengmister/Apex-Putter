import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Transform, TransformStamped
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.affines import compose, decompose

def htm_to_transform(htm: np.array) -> Transform:
    """
    Convert a HTM to a Transform object
    """
    # Decompose the result
    translation, rotation, _, _ = decompose(htm)
    quaternion = mat2quat(rotation)  # Returns w,x,y,z
    
    # Create and populate result transform
    result = Transform()
    result.translation.x = float(translation[0])
    result.translation.y = float(translation[1])
    result.translation.z = float(translation[2])
    result.rotation.w = float(quaternion[0])  # transforms3d returns w,x,y,z
    result.rotation.x = float(quaternion[1])
    result.rotation.y = float(quaternion[2])
    result.rotation.z = float(quaternion[3])

    return result

def combine_transforms(known_matrix: np.array, tag_transform: TransformStamped) -> Transform:
    """
    Combines a known transform matrix with a TF2 transform in ROS2
    
    Args:
        known_transform (4x4 np.array): A known homogeneous transformation matrix.
        tf2_transform (TransformStamped): Transform from TF2
        
    Returns:
        Transform: The resulting combined transform
    """

    # Convert TF2 transform to matrix
    tf2_translation = np.array([
        tag_transform.transform.translation.x,
        tag_transform.transform.translation.y,
        tag_transform.transform.translation.z
    ])
    tf2_rotation = quat2mat([
        tag_transform.transform.rotation.w,
        tag_transform.transform.rotation.x,
        tag_transform.transform.rotation.y,
        tag_transform.transform.rotation.z
    ])
    tf2_scale = np.ones(3)
    tf2_matrix = compose(tf2_translation, tf2_rotation, tf2_scale)
    
    # Combine transforms through matrix multiplication
    result_matrix = np.matmul(tf2_matrix, known_matrix)
    
    result = htm_to_transform(result_matrix)
    
    return result