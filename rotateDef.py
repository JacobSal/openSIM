import numpy as np

# ============================================================
# principal rotations


# rotation of phi is about the 1st axis (=X-axis) of the inertial frame
def r1(phi):
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), np.sin(phi)],
                     [0, -np.sin(phi), np.cos(phi)]])


# second rotation of theta is about the 2nd axis (=Y-axis) of the first intermediate frame
def r2(theta):
    return np.array([[np.cos(theta), 0, -np.sin(theta)],
                     [0, 1, 0],
                     [np.sin(theta), 0, np.cos(theta)]])


# third rotation of psi is about the 3rd axis (=Z-axis) of the second intermediate frame
def r3(psi):
    return np.array([[np.cos(psi), np.sin(psi), 0],
                     [-np.sin(psi), np.cos(psi), 0],
                     [0, 0, 1]])


# ====================================================
# e123 rotation sequence

def q11(psi, theta):
    return np.cos(psi) * np.cos(theta)


def q12(psi, theta, phi):
    return np.cos(psi) * np.sin(theta) * np.sin(phi) + np.sin(psi) * np.cos(phi)


def q13(psi, theta, phi):
    return -np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi)


def q21(psi, theta):
    return - np.sin(psi) * np.cos(theta)


def q22(psi, theta, phi):
    return -np.sin(psi) * np.sin(theta) * np.sin(phi) + np.cos(psi) * np.cos(phi)


def q23(psi, theta, phi):
    return np.sin(psi) * np.sin(theta) * np.cos(phi) + np.cos(psi) * np.sin(phi)


def q31(theta):
    return np.sin(theta)


def q32(theta, phi):
    return - np.cos(theta) * np.sin(phi)


def q33(theta, phi):
    return np.cos(theta) * np.cos(phi)


def e123_dcm(psi, theta, phi):
    return np.array([[q11(psi, theta), q12(psi, theta, phi), q13(psi, theta, phi)],
                     [q21(psi, theta), q22(psi, theta, phi), q23(psi, theta, phi)],
                     [q31(theta), q32(theta, phi), q33(theta, phi)]])

def getRotationMatrix(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix