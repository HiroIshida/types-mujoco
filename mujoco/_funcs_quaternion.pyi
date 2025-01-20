def mju_rotVecQuat(res: np.ndarray, vec: np.ndarray, quat: np.ndarray) -> None:
    """Rotate vector by quaternion."""
    ...

def mju_negQuat(res: np.ndarray, quat: np.ndarray) -> None:
    """Conjugate quaternion, corresponding to opposite rotation."""
    ...

def mju_mulQuat(res: np.ndarray, quat1: np.ndarray, quat2: np.ndarray) -> None:
    """Multiply quaternions."""
    ...

def mju_mulQuatAxis(res: np.ndarray, quat: np.ndarray, axis: np.ndarray) -> None:
    """Multiply quaternion and axis."""
    ...

def mju_axisAngle2Quat(res: np.ndarray, axis: np.ndarray, angle: float) -> None:
    """Convert axisAngle to quaternion."""
    ...

def mju_quat2Vel(res: np.ndarray, quat: np.ndarray, dt: float) -> None:
    """Convert quaternion (corresponding to orientation difference) to 3D velocity."""
    ...

def mju_subQuat(res: np.ndarray, qa: np.ndarray, qb: np.ndarray) -> None:
    """Subtract quaternions, express as 3D velocity: qb*quat(res) = qa."""
    ...

def mju_quat2Mat(res: np.ndarray, quat: np.ndarray) -> None:
    """Convert quaternion to 3D rotation matrix."""
    ...

def mju_mat2Quat(quat: np.ndarray, mat: np.ndarray) -> None:
    """Convert 3D rotation matrix to quaternion."""
    ...

def mju_derivQuat(res: np.ndarray, quat: np.ndarray, vel: np.ndarray) -> None:
    """Compute time-derivative of quaternion, given 3D rotational velocity."""
    ...

def mju_quatIntegrate(quat: np.ndarray, vel: np.ndarray, scale: float) -> None:
    """Integrate quaternion given 3D angular velocity."""
    ...

def mju_quatZ2Vec(quat: np.ndarray, vec: np.ndarray) -> None:
    """Construct quaternion performing rotation from z-axis to given vector."""
    ...

def mju_euler2Quat(quat: np.ndarray, euler: np.ndarray, seq: str) -> None:
    """Convert sequence of Euler angles (radians) to quaternion. seq[0,1,2] must be in ‘xyzXYZ’, lower/upper-case mean intrinsic/extrinsic rotations."""
    ...
