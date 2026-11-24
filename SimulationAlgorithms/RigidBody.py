# rigid_body.py
import numpy as np


def euler_to_rot_body_to_world(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


class RigidBody:
    def __init__(
        self,
        mass=1.2,
        inertia=None,
        pos=None,
        vel=None,
        rpy=None,
        omega=None,
        gravity=9.81,
    ):
        self.mass = float(mass)
        self.gravity = float(gravity)

        # 3x3 moments of inertia matrix (diagonal by default)
        if inertia is None:
            self.inertia = np.diag([0.015, 0.015, 0.025])
        else:
            self.inertia = np.array(inertia, dtype=float).reshape(3, 3)

        self.inv_inertia = np.linalg.inv(self.inertia)

        # States
        self.pos = np.zeros(3, dtype=float) if pos is None else np.array(pos, dtype=float)
        self.vel = np.zeros(3, dtype=float) if vel is None else np.array(vel, dtype=float)
        self.rpy = np.zeros(3, dtype=float) if rpy is None else np.array(rpy, dtype=float)      # [roll, pitch, yaw]
        self.omega = np.zeros(3, dtype=float) if omega is None else np.array(omega, dtype=float)  # [p, q, r] body rates

        # Most recent world acceleration (useful for IMU specific force)
        self.accel = np.zeros(3, dtype=float)

    def step(self, force_world, torque_body, dt):
        force_world = np.array(force_world, dtype=float)
        torque_body = np.array(torque_body, dtype=float)

        # 1. Translation (Newton's 2nd law + gravity in world frame, Z up)
        gravity_force = np.array([0.0, 0.0, -self.mass * self.gravity])
        self.accel = (force_world + gravity_force) / self.mass
        self.vel = self.vel + self.accel * dt
        self.pos = self.pos + self.vel * dt

        # 2. Rotation (Euler's rotational equation in body frame)
        # J * omega_dot + omega x (J * omega) = torque_body
        j_omega = self.inertia @ self.omega
        gyro_torque = np.cross(self.omega, j_omega)
        omega_dot = self.inv_inertia @ (torque_body - gyro_torque)
        self.omega = self.omega + omega_dot * dt

        # 3. Convert body angular rates [p, q, r] to Euler angle rates [roll_dot, pitch_dot, yaw_dot]
        roll, pitch, _ = self.rpy
        sr, cr = np.sin(roll), np.cos(roll)
        tp = np.tan(pitch)
        cp = max(np.cos(pitch), 1e-3)
        p, q, r = self.omega

        rpy_dot = np.array([
            p + sr * tp * q + cr * tp * r,
            cr * q - sr * r,
            (sr / cp) * q + (cr / cp) * r,
        ])
        self.rpy = self.rpy + rpy_dot * dt
        self.rpy[2] = (self.rpy[2] + np.pi) % (2.0 * np.pi) - np.pi