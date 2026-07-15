# -*- coding: utf-8 -*-
# Author: Nuofan - 12233200@mail.sustech.edu.cn
"""
contains functions for math operations,
including matrix operations, spacial mathematics, etc
"""
import adsk, adsk.core, adsk.fusion
import math

def matrix3d_2_pose(matrix: adsk.core.Matrix3D):
    """
    Convert configuration matrix (Matrix3D) into pose(translation, rotation) representation

    Parameter:
    ---------
    matrix: adsk.core.Matric3D
        Homogeneous matrix

    Return: 
    ---------
    pose: List
        [x, y, z, roll, pitch, yaw] 
        This euler angle convertion is "XYZ"
    """
    x = matrix.translation.x * 0.01 # cm to m
    y = matrix.translation.y * 0.01 # cm to m
    z = matrix.translation.z * 0.01 # cm to m

    threshold = 10e-10
    check = lambda v: v if abs(v) > threshold else 0.0

    r11 = matrix.getCell(0, 0)
    r21 = matrix.getCell(1, 0)
    r31 = matrix.getCell(2, 0)
    r32 = matrix.getCell(2, 1)
    r33 = matrix.getCell(2, 2)
    r23 = matrix.getCell(1, 2)
    r22 = matrix.getCell(1, 1)
    sy = math.sqrt(r11**2 + r21**2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(r32, r33)
        pitch = math.atan2(-r31, sy)
        yaw = math.atan2(r21, r11)
    else: 
        roll = math.atan2(-r23, r22)
        pitch = math.atan2(-r31, sy)
        yaw = 0

    pose = [x, y, z, roll, pitch, yaw]
    return pose

def matrix3d_2_euler_xyz(matrix: adsk.core.Matrix3D):
    """
    Convert configuration matrix (Matrix3D) into pose(translation, rotation) representation

    Parameter:
    ---------
    matrix: adsk.core.Matric3D
        Homogeneous matrix

    Return: 
    ---------
    pose: List
        [x, y, z, roll, pitch, yaw] 
        This euler angle convertion is "xyz"
    """

    # TODO: Still needs to check if it is right
    # Some problem seems related with this function
    x = matrix.translation.x * 0.01 # cm to m
    y = matrix.translation.y * 0.01 # cm to m
    z = matrix.translation.z * 0.01 # cm to m

    threshold = 10e-10
    check = lambda v: v if abs(v) > threshold else 0.0

    r11 = matrix.getCell(0, 0)
    r12 = matrix.getCell(0, 1)
    r13 = matrix.getCell(0, 2)
    r22 = matrix.getCell(1, 1)
    r23 = matrix.getCell(1, 2)
    r32 = matrix.getCell(2, 1)
    r33 = matrix.getCell(2, 2)

    sy = math.sqrt(r33**2 + r23**2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(-r23, r33)
        pitch = math.atan2(r13, sy)
        yaw = math.atan2(-r12, r11)
    else:
        roll = math.atan2(-r32, r22)
        pitch = math.atan2(r13, sy)
        yaw = 0
    
    pose = [x, y, z, roll, pitch, yaw]
    return pose 

def coordinate_transform(w_T_from: adsk.core.Matrix3D, w_T_to: adsk.core.Matrix3D) -> adsk.core.Matrix3D:
    """
    Returns a transform matrix to transform a from_frame to to_frame
    Parameters
    ---------
    w_T_from: the from_frame
    w_T_to: the to_frame
    Return
    ---------
    from_T_to: adsk.core.Matrix3D
        represent to_frame w.r.t from_frame
        w_T_from * from_T_to = w_T_to       ->
        from_T_to = inv(w_T_from) * w_T_to
    """ 
    from_T_to = adsk.core.Matrix3D.create()
    # invert a copy: invert() works in place and would silently corrupt the
    # caller's matrix (e.g. a Link.pose that is reused later in the export)
    from_T_w = w_T_from.copy()
    from_T_w.invert()

    for i in range(4):
        for j in range(4):
            value = 0
            for k in range(4):
                value += from_T_w.getCell(i, k) * w_T_to.getCell(k, j)
            from_T_to.setCell(i, j, value)

    return from_T_to

def change_orientation(a_R_b: list, b_v: list):
    """
    Rotate vector v to a different orientation or
    Represent vector v in a different frame

    Parameters
    ---------
    a_R_b: 3*3 array
        represent b-frame's orientation w.r.t a-frame
    b_v: 3*1 array
        v's coordinates w.r.t b-frame
    Return:
    ---------
    a_v: 3*1 array
        v's coordinates w.r.t a-frame
    """
    a_v = [[a_R_b[0][0]*b_v[0][0]+a_R_b[0][1]*b_v[1][0]+a_R_b[0][2]*b_v[2][0]],
           [a_R_b[1][0]*b_v[0][0]+a_R_b[1][1]*b_v[1][0]+a_R_b[1][2]*b_v[2][0]],
           [a_R_b[2][0]*b_v[0][0]+a_R_b[2][1]*b_v[1][0]+a_R_b[2][2]*b_v[2][0]]]
    
    return a_v

def matrix3D_to_str(M: adsk.core.Matrix3D) -> str:
    """
    Return a str for printing Matrix3D object
    """
    s = "[{}, {}, {}, {}\n \
{}, {}, {}, {}\n \
{}, {}, {}, {}\n \
{}, {}, {}, {}\n]".format(M.getCell(0,0), M.getCell(0,1), M.getCell(0,2), M.getCell(0,3),
                                  M.getCell(1,0), M.getCell(1,1), M.getCell(1,2), M.getCell(1,3),
                                  M.getCell(2,0), M.getCell(2,1), M.getCell(2,2), M.getCell(2,3),
                                  M.getCell(3,0), M.getCell(3,1), M.getCell(3,2), M.getCell(3,3))
    
    return s

def matrix_multi(M1: list, M2: list):
    """
    Return M = M1 * M2

    Parameters:
    ---------
    M1: 3*3 list
    M2: 3*3 list

    Return:
    ---------
    M: 3*3 list
    """
    M = [[0 for i in range(3)] for j in range(3)] # generate a 3*3 list

    for i in range(3):
        for j in range(3):
            for k in range(3):
                M[i][j] += M1[i][k]*M2[k][j]

    return M

def get_rotation_matrix(H: adsk.core.Matrix3D):
    """
    Get rotation matrix R from Matrix3D object

    Parameters:
    ---------
    H: adsk.core.Matrix3D
    
    Return:
    ---------
    R: 3*3 list
        Rotation matrix
    """
    R = [[H.getCell(0, 0), H.getCell(0, 1), H.getCell(0, 2)],
         [H.getCell(1, 0), H.getCell(1, 1), H.getCell(1, 2)],
         [H.getCell(2, 0), H.getCell(2, 1), H.getCell(2, 2)]]
    
    return R

def matrix_transpose(M: list):
    """
    Return transpose of a matrix

    Parameter:
    ---------
    M: 3*3 list

    Return:
    ---------
    M_T: 3*3 list
        Transpose of M
    """
    M_T = [[0 for i in range(3)] for j in range(3)] # generate a 3*3 list

    for i in range(3):
        for j in range(3):
            M_T[i][j] = M[j][i]

    return M_T

def matrix_multiply(A, B):
    # Matrix multiplication of A and B
    result = [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]
    return result

def matrix_subtract(A, B):
    # Matrix subtraction of A and B
    result = [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    return result

def determinant_2x2(matrix):
    # Calculate determinant of a 2x2 matrix
    return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]

def determinant_3x3(matrix):
    # Calculate determinant of a 3x3 matrix
    det = matrix[0][0] * determinant_2x2([[matrix[1][1], matrix[1][2]], [matrix[2][1], matrix[2][2]]])
    det -= matrix[0][1] * determinant_2x2([[matrix[1][0], matrix[1][2]], [matrix[2][0], matrix[2][2]]])
    det += matrix[0][2] * determinant_2x2([[matrix[1][0], matrix[1][1]], [matrix[2][0], matrix[2][1]]])
    return det

def eigenvalues_and_eigenvectors(matrix):
    # Finding eigenvalues and eigenvectors of a 3x3 matrix
    detA = determinant_3x3(matrix)
    A_minus_lambdaI = matrix_subtract(matrix, [[detA, 0, 0], [0, detA, 0], [0, 0, detA]])

    eigenvalues = []
    eigenvectors = []

    # Finding the eigenvalues
    eigenvalues.append(detA - matrix[0][0])
    eigenvalues.append(detA - matrix[1][1])
    eigenvalues.append(detA - matrix[2][2])

    # Finding the eigenvectors for each eigenvalue
    for eig_val in eigenvalues:
        A_minus_lambdaI[0][0] = matrix[0][0] - eig_val
        A_minus_lambdaI[1][1] = matrix[1][1] - eig_val
        A_minus_lambdaI[2][2] = matrix[2][2] - eig_val

        # Solve the linear system (A-lambdaI)v = 0
        v = [0, 0, 0]
        if A_minus_lambdaI[0][0] == 0 and A_minus_lambdaI[0][1] == 0 and A_minus_lambdaI[0][2] == 0:
            v[0] = 1
        elif A_minus_lambdaI[1][0] == 0 and A_minus_lambdaI[1][1] == 0 and A_minus_lambdaI[1][2] == 0:
            v[1] = 1
        elif A_minus_lambdaI[2][0] == 0 and A_minus_lambdaI[2][1] == 0 and A_minus_lambdaI[2][2] == 0:
            v[2] = 1
        else:
            v[0] = -(A_minus_lambdaI[0][1]*A_minus_lambdaI[1][2] - A_minus_lambdaI[0][2]*A_minus_lambdaI[1][1])
            v[1] = A_minus_lambdaI[0][0]*A_minus_lambdaI[1][2] - A_minus_lambdaI[0][2]*A_minus_lambdaI[1][0]
            v[2] = -(A_minus_lambdaI[0][0]*A_minus_lambdaI[1][1] - A_minus_lambdaI[0][1]*A_minus_lambdaI[1][0])
        
        # Normalize the eigenvector
        magnitude = (v[0]**2 + v[1]**2 + v[2]**2)**0.5
        eigenvectors.append([v[0] / magnitude, v[1] / magnitude, v[2] / magnitude])

    return eigenvalues, eigenvectors