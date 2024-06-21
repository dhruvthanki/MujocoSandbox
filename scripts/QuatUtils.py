import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import logm

def quaternion_conjugate(q:np.ndarray)->np.ndarray:
    """ Return the conjugate of a quaternion q """
    x, y, z, w = q
    return np.array([-x, -y, -z, w])

def quaternion_multiply(q1:np.ndarray, q2:np.ndarray)->np.ndarray:
    """ Multiply two quaternions q1 and q2 """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([x, y, z, w])

def quaternion_to_rotation_vector(q:np.ndarray)->np.ndarray:
    """ Convert a quaternion to a rotation vector """
    x, y, z, w = q
    angle = 2 * np.arccos(w)
    s = np.sqrt(1 - w*w)
    if s < 1e-8:
        return np.array([x, y, z])
    else:
        return np.array([x / s, y / s, z / s]) * angle

def shortest_error_quaternion(desired_q:np.ndarray, current_q:np.ndarray)->np.ndarray:
    """ Compute the shortest error between two quaternions """
    # q_error = quaternion_multiply(quaternion_conjugate(current_q), desired_q)
    q_error = quaternion_multiply(desired_q, quaternion_conjugate(current_q))
    rotation_vector = quaternion_to_rotation_vector(q_error)
    return rotation_vector

def quaternion_log_map(q):
    # Normalize the quaternion
    q = q / np.linalg.norm(q)
    
    # Extract the scalar and vector parts
    w, x, y, z = q
    vec_part = np.array([x, y, z])
    
    # Compute the angle theta
    theta = np.arccos(w)
    
    # Compute the log map
    if np.isclose(theta, 0):
        return np.zeros(3)
    else:
        return theta * vec_part / np.sin(theta)

def skew_symmetric_to_vector(skew_matrix):
    return np.array([skew_matrix[2, 1], skew_matrix[0, 2], skew_matrix[1, 0]])

def quaternion_to_log_vector(q):
    # Normalize the quaternion
    q = q / np.linalg.norm(q)
    
    # Convert quaternion to rotation matrix
    rotation = R.from_quat(q)
    rot_matrix = rotation.as_matrix()
    
    # # Calculate the log of the rotation matrix
    # # Using the formula log(R) = θ / 2sin(θ) * (R - R.T)
    # theta = np.arccos((np.trace(rot_matrix) - 1) / 2)
    # if np.isclose(theta, 0):
    #     return np.zeros(3)
    
    # log_rot_matrix = theta / (2 * np.sin(theta)) * (rot_matrix - rot_matrix.T)

    log_rot_matrix = logm(rot_matrix)
    
    # Extract the vector from the skew-symmetric matrix
    log_vector = skew_symmetric_to_vector(log_rot_matrix)
    
    return log_vector

if __name__ == "__main__":
    # Example usage:
    desired_q = np.array([0.0, 0.7071, 0.0, 0.7071])  # Example desired quaternion
    current_q = np.array([0.0, 0.0, 0.0, 1.0])        # Example current quaternion

    error_vector = shortest_error_quaternion(desired_q, current_q)
    print("Shortest error (rotation vector):", error_vector)

    error_vector = shortest_error_quaternion(current_q, desired_q)
    print("Shortest error (rotation vector):", error_vector)

    # # q_error = quaternion_multiply(quaternion_conjugate(current_q), desired_q)
    # q_error = quaternion_multiply(desired_q, quaternion_conjugate(current_q))

    # error_vector = quaternion_log_map(q_error)
    # print("Error (rotation vector):", error_vector)

    # log_vector = quaternion_to_log_vector(q_error)
    # print("Log Vector:", log_vector)