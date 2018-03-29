#!/usr/bin/env python

import sys, numpy, math
import rospy
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.srv import *
from odometry_cnn_data import load_kitti_poses, Odometry

SERVICE_NAME = 'compute_effector_camera_quick'

def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix:

    r1 r2 r3 0
    r4 r5 r6 0
    r7 r8 r9 0
    tx ty tz 1

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    >>> q = quaternion_from_matrix(numpy.identity(4), True)
    >>> numpy.allclose(q, [1, 0, 0, 0])
    True
    >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
    >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R, True)
    >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q = quaternion_from_matrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True
    >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
    ...                    quaternion_from_matrix(R, isprecise=True))
    True
    >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
    >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
    ...                    quaternion_from_matrix(R, isprecise=True))
    True

    """
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    if isprecise:
        q = numpy.empty((4, ))
        t = numpy.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = numpy.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = numpy.linalg.eigh(K)
        q = V[[3, 0, 1, 2], numpy.argmax(w)]
    if q[0] < 0.0:
        numpy.negative(q, q)
    return q


def pose_to_ros_transform(pose):
    t = Transform()
    t.translation.x = pose.dof[0]
    t.translation.y = pose.dof[1]
    t.translation.z = pose.dof[2]

    q = quaternion_from_matrix(pose.M)
    t.rotation.w = q[0]
    t.rotation.x = q[1]
    t.rotation.y = q[2]
    t.rotation.z = q[3]

    return t


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.stderr.write("ERROR, expecting two pose files!\n")
        sys.exit(1)

    poses1 = load_kitti_poses(sys.argv[1])[0:50]
    # translation:
    # x: 0.0113782870643
    # y: 0.118657435794
    # z: 0.173699919874
    # rotation:
    # x: 0.0362682036379
    # y: -0.803976727751
    # z: 0.593376488207
    # w: 0.0145045469662
    c = Odometry([-0.996948, -0.075531, 0.019719, 0.011378,
                  -0.041104, 0.293178, -0.955174, 0.118658,
                  0.066364, -0.953070, -0.295388, 0.173698])
    poses2t = [c.inv() * p * c for p in poses1]
    poses2 = load_kitti_poses(sys.argv[2])[0:50]

    assert len(poses1) == len(poses2)

    request = compute_effector_camera_quickRequest()
    for p1,p2,p2t in zip(poses1, poses2, poses2t):
        print p1.dof, "\n", p2.dof, "\n", p2t.dof, "\n--------------------------------------------------------------------------------"
        request.world_effector.transforms.append(pose_to_ros_transform(p1))
        request.camera_object.transforms.append(pose_to_ros_transform(p2.inv()))

    rospy.wait_for_service(SERVICE_NAME)
    try:
        service = rospy.ServiceProxy(SERVICE_NAME, compute_effector_camera_quick)
        calibration = service(request)
        print calibration
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
