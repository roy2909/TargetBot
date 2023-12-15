"""Test quaternion."""
import numpy as np
from gun_trajectory.quaternion import euler_to_quaternion, quaternion_multiply
from geometry_msgs.msg import Quaternion


def test_quaternion():
    quat = euler_to_quaternion(np.pi, 0, 0)
    quat.x = round(quat.x, 3)
    quat.y = round(quat.y, 3)
    quat.z = round(quat.z, 3)
    quat.w = round(quat.w, 3)
    assert quat == Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    quat1 = euler_to_quaternion(np.pi/2, 0, 0)
    quat1.x = round(quat1.x, 3)
    quat1.y = round(quat1.y, 3)
    quat1.z = round(quat1.z, 3)
    quat1.w = round(quat1.w, 3)
    assert quat1 == Quaternion(x=0.707, y=0.0, z=0.0, w=0.707)

    mult_quat = quaternion_multiply(quat, quat1)
    mult_quat.x = round(mult_quat.x, 3)
    mult_quat.y = round(mult_quat.y, 3)
    mult_quat.z = round(mult_quat.z, 3)
    mult_quat.w = round(mult_quat.w, 3)
    assert mult_quat == Quaternion(x=0.707, y=0.0, z=0.0, w=-0.707)
