import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Transform, TransformStamped
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.affines import compose, decompose
from geometry_msgs.msg import Pose

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

def transform_to_htm(transform: Transform) -> np.array:
    '''
    Args:
        transform: Transfrom is msg type of tf publisher.
    Returns:
        htm format of the same (4x4 np.array): homogeneous transformation matrix 
    '''
    translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ])
    quaternion = np.array([
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z
    ])
    rotation_matrix = quat2mat(quaternion)
    htm = np.eye(4)
    htm[0:3, 0:3] = rotation_matrix 
    htm[0:3, 3] = translation 
    return htm

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

def obj_in_bot_frame(T_camObj):
    '''
    Input(4x4 np.array): Camera-to-Ball transform  T_camObj
    Output(4x4 np.array): T_objBot
    Fixed: T_botCam    
    '''
    # Write the fixed frame transform here.
    T_botCam = np.array([0])
    T_objBot = np.dot(np.linalg.inv(T_camObj),np.linalg.inv(T_botCam))
    return T_objBot

def detected_obj_pose(T_camObj: Transform):
    '''
    This function returns the pose in robot frame of the robot to 
    reach the object detected by vision.

    Args: 
        transform: tf of object detected in camera frame
    Returns:
        pose: pose(or waypoint) of the object in robot frame.
    '''

    T_camObj = transform_to_htm(T_camObj)
    T_objBot = obj_in_bot_frame(T_camObj)
    pose = Pose()
    pose.position.x = T_objBot[0,3]
    pose.position.y = T_objBot[1,3]
    pose.position.z = T_objBot[2,3]
    # Figure a way to calc optimal orientation
    # pose.orientation.x = 0.90305
    # pose.orientation.y = 0.429622
    # pose.orientation.z = -3.8634e-05
    # pose.orientation.w = -5.0747e-06
    return pose


# Test functions
def test():
    manipulator_pos = np.array([
        [0.70710678, -0.70710678, 0, 0],
        [0.70710678, 0.70710678, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    tranfrom = htm_to_transform(manipulator_pos)
    htm = transform_to_htm(tranfrom)
    print(htm)


test()
