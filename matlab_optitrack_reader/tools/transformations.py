# Hacky subset of Baxter Commander
# https://github.com/baxter-flowers/baxter_commander/blob/master/src/transformations/transformations.py

import numpy
from . tf_transformations import *

def _is_indexable(var):
    try:
        var[0]
    except TypeError:
        return False
    except IndexError:
        return True
    return True

def multiply_transform(t1, t2):
    """
    Combines two transformations together
    The order is translation first, rotation then
    :param t1: [[x, y, z], [x, y, z, w]] or matrix 4x4
    :param t2: [[x, y, z], [x, y, z, w]] or matrix 4x4
    :return: The combination t1-t2 in the form [[x, y, z], [x, y, z, w]] or matrix 4x4
    """
    if _is_indexable(t1) and len(t1) == 2:
        t1m = translation_matrix(t1[0])
        r1m = quaternion_matrix(t1[1])
        m1m = numpy.dot(t1m, r1m)
        t2m = translation_matrix(t2[0])
        r2m = quaternion_matrix(t2[1])
        m2m = numpy.dot(t2m, r2m)
        rm = numpy.dot(m1m, m2m)
        rt = translation_from_matrix(rm)
        rr = quaternion_from_matrix(rm)
        return [list(rt), list(rr)]
    else:
        return numpy.dot(t1, t2)

def inverse_transform(t):
    """
    Return the inverse transformation of t
    :param t: A transform [[x, y, z], [x, y, z, w]]
    :return: t2 such as multiply_transform_(t, t2) = [[0, 0, 0], [0, 0, 0, 1]]
    """
    return [quat_rotate(quaternion_inverse(t[1]), [-t[0][0], -t[0][1], -t[0][2]]), quaternion_inverse(t[1])]

def quat_rotate(rotation, vector):
    """
    Rotate a vector according to a quaternion. Equivalent to the C++ method tf::quatRotate
    :param rotation: the rotation
    :param vector: the vector to rotate
    :return: the rotated vector
    """
    def quat_mult_quat(q1, q2):
        return (q1[0]*q2[0], q1[1]*q2[1], q1[2]*q2[2], q1[3]*q2[3])

    def quat_mult_point(q, w):
        return (q[3] * w[0] + q[1] * w[2] - q[2] * w[1],
                q[3] * w[1] + q[2] * w[0] - q[0] * w[2],
                q[3] * w[2] + q[0] * w[1] - q[1] * w[0],
                -q[0] * w[0] - q[1] * w[1] - q[2] * w[2])

    q = quat_mult_point(rotation, vector)
    q = quaternion_multiply(q, quaternion_inverse(rotation))
    return [q[0], q[1], q[2]]
