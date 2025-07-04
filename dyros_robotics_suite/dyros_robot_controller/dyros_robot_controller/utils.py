import numpy as np
import threading
import ctypes

def cubic_spline(
    time: float,
    time_0: float,
    time_f: float,
    x_0: np.ndarray,
    x_f: np.ndarray,
    x_dot_0: np.ndarray,
    x_dot_f: np.ndarray
) -> np.ndarray:
    """
    Computes a cubic spline interpolation for a given time within an interval.

    This function performs cubic spline interpolation between an initial state x_0 at time_0 and
    a final state x_f at time_f, given the corresponding initial and final derivatives (velocities)
    x_dot_0 and x_dot_f. If the specified time is outside the interval [time_0, time_f], it returns
    the respective boundary values.

    Parameters:
        time (float): The current time at which to evaluate the spline.
        time_0 (float): The starting time of the interpolation interval.
        time_f (float): The ending time of the interpolation interval.
        x_0 (np.ndarray): The starting value (state) at time_0.
        x_f (np.ndarray): The ending value (state) at time_f.
        x_dot_0 (np.ndarray): The derivative (velocity) at time_0.
        x_dot_f (np.ndarray): The derivative (velocity) at time_f.

    Returns:
        np.ndarray: The interpolated state at the given time.
    """
    # If current time is before the start time, return the initial state.
    if time < time_0:
        return x_0
    # If current time is after the final time, return the final state.
    elif time > time_f:
        return x_f
    else:
        # Total duration of the interpolation interval.
        T = time_f - time_0
        # Elapsed time from the start.
        t = time - time_0
        # Coefficients for the cubic polynomial:
        # a0: initial state
        a0 = x_0
        # a1: initial derivative
        a1 = x_dot_0
        # a2: second coefficient computed from boundary conditions
        a2 = ( 3*(x_f - x_0) - T*(2*x_dot_0 + x_dot_f) ) / (T**2)
        # a3: third coefficient computed from boundary conditions
        a3 = ( -2*(x_f - x_0) + T*(x_dot_0 + x_dot_f) ) / (T**3)
        # Return the value of the cubic polynomial at time t.
        return a0 + a1*t + a2*(t**2) + a3*(t**3)

def cubic_dot_spline(
    time: float,
    time_0: float,
    time_f: float,
    x_0: np.ndarray,
    x_f: np.ndarray,
    x_dot_0: np.ndarray,
    x_dot_f: np.ndarray
) -> np.ndarray:
    """
    Computes the derivative of a cubic spline interpolation for a given time within an interval.

    This function computes the derivative of the cubic spline interpolation defined in cubic_spline()
    for a given time within the specified interval. It returns the derivative at the given time.

    Parameters:
        time (float): The current time at which to evaluate the derivative.
        time_0 (float): The starting time of the interpolation interval.
        time_f (float): The ending time of the interpolation interval.
        x_0 (np.ndarray): The starting value (state) at time_0.
        x_f (np.ndarray): The ending value (state) at time_f.
        x_dot_0 (np.ndarray): The derivative (velocity) at time_0.
        x_dot_f (np.ndarray): The derivative (velocity) at time_f.

    Returns:
        np.ndarray: The derivative of the interpolated state at the given time.
    """
    # If current time is before the start time, return the initial derivative.
    if time < time_0:
        return x_dot_0
    # If current time is after the final time, return the final derivative.
    elif time > time_f:
        return x_dot_f
    else:
        # Total duration of the interpolation interval.
        T = time_f - time_0
        # Elapsed time from the start.
        t = time - time_0
        # Coefficients for the cubic polynomial derivatives:
        # b0: initial derivative
        b0 = x_dot_0
        # b1: second coefficient computed from boundary conditions
        b1 = 2*(3*(x_f - x_0) - T*(2*x_dot_0 + x_dot_f)) / (T**2)
        # b2: third coefficient computed from boundary conditions
        b2 = 3*( -2*(x_f - x_0) + T*(x_dot_0 + x_dot_f) ) / (T**3)
        # Return the value of the cubic polynomial derivative at time t.
        return b0 + b1*t + b2*(t**2)

def _vec_to_skew(ω: np.ndarray) -> np.ndarray:
    """
    Convert a 3-vector to a 3x3 skew-symmetric matrix.

    Parameters
    ----------
    ω : (3,) ndarray
        Axis-angle vector.

    Returns
    -------
    (3,3) ndarray
        Skew-symmetric matrix so that (_vec_to_skew(ω) @ v) = ω x v.
    """
    return np.array([[ 0.0, -ω[2],  ω[1]],
                     [ ω[2], 0.0,  -ω[0]],
                     [-ω[1], ω[0],  0.0]])


def _skew_to_vec(S: np.ndarray) -> np.ndarray:
    """
    Inverse of _vec_to_skew.

    Parameters
    ----------
    S : (3,3) ndarray
        Skew-symmetric matrix

    Returns
    -------
    (3,) ndarray
        Corresponding vector.
    """
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def so3_exp(ω: np.ndarray) -> np.ndarray:
    """
    Exponential map from so(3) → SO(3).

    Parameters
    ----------
    ω : (3,) ndarray
        Axis-angle vector.

    Returns
    -------
    (3,3) ndarray
        Rotation matrix R = exp([ω]x).
    """
    θ = np.linalg.norm(ω)
    if θ < 1e-12:               # Small-angle approximation
        return np.eye(3) + _vec_to_skew(ω)
    A = np.sin(θ) / θ
    B = (1.0 - np.cos(θ)) / θ**2
    K = _vec_to_skew(ω)
    return np.eye(3) + A * K + B * (K @ K)


def so3_log(R: np.ndarray) -> np.ndarray:
    """
    Logarithm map SO(3) → so(3).

    Parameters
    ----------
    R : (3,3) ndarray
        Rotation matrix.

    Returns
    -------
    (3,3) ndarray
        Skew-symmetric matrix S such that exp(S) == R
        (vector form obtainable with _skew_to_vec).
    """
    # Clamp trace for numerical stability
    trace_R = np.clip(np.trace(R), -1.0, 3.0)
    θ = np.arccos((trace_R - 1.0) / 2.0)
    if θ < 1e-12:               # R ≈ I
        return np.zeros((3, 3))
    return θ / (2.0 * np.sin(θ)) * (R - R.T)

def rotation_cubic(time: float, 
                   time_0: float,  
                   time_f: float,
                   R_0: np.ndarray, 
                   R_f: np.ndarray) -> np.ndarray:
    """Kang-&-Park cubic rotation interpolation using your cubic_spline."""
    if time <= time_0:
        return R_0
    if time >= time_f:
        return R_f

    # Relative rotation in the Lie algebra
    r_skew = so3_log(R_0.T @ R_f)        # 3×3 skew
    r_vec  = _skew_to_vec(r_skew)      # 3-vector

    # Vector τ (each element goes 0 → 1)
    tau_vec = cubic_spline(
        time, time_0, time_f,
        np.zeros(3), np.ones(3),
        np.zeros(3), np.zeros(3)
    )

    # R(t) = R0 · exp([τ·r]×)
    return R_0 @ so3_exp(r_vec * tau_vec)


def rotation_cubic_dot(time: float, 
                       time_0: float, 
                       time_f: float,
                       R_0: np.ndarray, 
                       R_f: np.ndarray) -> np.ndarray:
    """Angular velocity in the space frame."""
    r_skew = so3_log(R_0.T @ R_f)
    r_vec  = _skew_to_vec(r_skew)

    # Derivative of τ · r with your cubic_dot_spline
    rd = cubic_dot_spline(
        time, time_0, time_f,
        np.zeros(3), r_vec,
        np.zeros(3), np.zeros(3)
    )

    # ω = R(t) · ṙ
    return rotation_cubic(time, time_0, time_f, R_0, R_f) @ rd

import numpy as np

def get_phi(current_rotation: np.ndarray, desired_rotation: np.ndarray) -> np.ndarray:
    """
    Compute the orientation error vector φ between two rotation matrices.

    This is the direct NumPy equivalent of the Eigen-based C++ routine:

        φ = -½ ∑_{i=0}^{2} ( R_cur.col(i) x R_des.col(i) )

    A small-angle controller can use ω = K φ to steer the current rotation
    toward the desired one.

    Parameters
    ----------
    current_rotation : (3, 3) ndarray
        Current orientation matrix R_cur.
    desired_rotation : (3, 3) ndarray
        Desired orientation matrix R_des.

    Returns
    -------
    (3,) ndarray
        Orientation error vector φ (in the *space* frame).
    """
    # Basic shape check
    if current_rotation.shape != (3, 3) or desired_rotation.shape != (3, 3):
        raise ValueError("Both inputs must be 3x3 rotation matrices.")

    # Column-wise cross products v_i × w_i for i = 0,1,2
    # np.cross with axis=0 treats each column as a vector
    phi = 0
    for col in range(3):
        phi += np.cross(current_rotation[:, col], desired_rotation[:, col])

    phi *= -0.5

    return phi


class ControlledThread(threading.Thread):
    """
    A custom thread class that allows controlled execution with the ability to forcibly terminate
    the thread if it exceeds a specified time limit.

    This class wraps a target function and executes it in a separate thread while holding a lock.
    The kill() method uses Python's C API to asynchronously raise a SystemExit exception in the thread,
    attempting to terminate its execution.
    """

    def __init__(self, target, *args, **kwargs):
        """
        Initializes the ControlledThread.

        Parameters:
            target (callable): The function to be executed in the thread.
            *args: Variable length argument list for the target function.
            **kwargs: Arbitrary keyword arguments for the target function.

        Returns:
            None
        """
        super().__init__()
        # Store the target function and its arguments.
        self._target = target
        self._args = args
        self._kwargs = kwargs
        
        # Lock to control access during the thread execution.
        self.calculation_lock = threading.Lock()

    def run(self):
        """
        Runs the target function within a controlled (locked) context.

        This method overrides threading.Thread.run() and acquires a lock before
        executing the target function to protect shared resources.

        Returns:
            None
        """
        # Acquire the lock to ensure the target function runs in a protected context.
        with self.calculation_lock:
            self._target(*self._args, **self._kwargs)

    def kill(self):
        """
        Attempts to terminate the thread by asynchronously raising a SystemExit exception.

        This method uses ctypes to access Python's C API and inject an exception into the thread.
        If more than one thread is affected by the exception injection, it will revert the injection.

        Returns:
            None
        """
        # Get the thread identifier.
        tid = self.ident
        if not tid:
            return
        # Attempt to asynchronously raise SystemExit in the thread.
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(tid),
            ctypes.py_object(SystemExit)
        )
        # If more than one thread was affected, revert the exception injection.
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), None)

def LowPassFilter(input, prev_res, sampling_freq, cutoff_freq):
        rc = 1. / (cutoff_freq * 2 * np.pi)
        dt = 1. / sampling_freq
        a = dt / (rc + dt)
        return prev_res + a * (input - prev_res)