import numpy as np
from geometry_msgs.msg import Quaternion


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.

    Args:
    ----
        roll (float): Rotation around the x-axis (roll) in radians.
        pitch (float): Rotation around the y-axis (pitch) in radians.
        yaw (float): Rotation around the z-axis (yaw) in radians.

    Returns
    -------
        Quaternion: The quaternion representing the input Euler angles.

    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    quat = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return quat


# https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr


def quaternion_multiply(q0, q1):
    # example adapted from ROS2 docs
    """
    Multiplies two quaternions.

    Input
    -----
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    ------
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0.w
    x0 = q0.x
    y0 = q0.y
    z0 = q0.z

    # Extract the values from q1
    w1 = q1.w
    x1 = q1.x
    y1 = q1.y
    z1 = q1.z

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = Quaternion(w=q0q1_w, x=q0q1_x, y=q0q1_y, z=q0q1_z)

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion
