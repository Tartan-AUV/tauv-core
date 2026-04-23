"""ROS2 node that plans smooth odometry trajectories from current to target state.

The planner generates a time-parameterized quintic polynomial trajectory to reach the
target position, orientation and velocity from the current state. 

The trajectory is sampled at a fixed period and published as a queue of setpoints to be
consumed by a downstream controller. The planner dispatches the next setpoint when the
current setpoint has been reached within configured tolerances.

The planner runs whenever a new target odometry message is received, replanning from the
current state to the new target.

The planner also checks whether the current setpoint has been reached at a fixed dispatch rate, and advances
the setpoint queue accordingly.

Inputs:
- Current vehicle state (`nav_msgs/Odometry`)
- Target vehicle state (`nav_msgs/Odometry`)

Outputs:
- Dispatched controller setpoints as `nav_msgs/Odometry` points
"""

from __future__ import annotations

import copy
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node


@dataclass(frozen=True)
class QuinticBoundary:
    """Boundary conditions for one scalar quintic trajectory component."""

    p0: float
    v0: float
    a0: float
    p1: float
    v1: float
    a1: float


class TrajectoryPlannerNode(Node):
    """Builds a smooth, time-parameterized trajectory between two odometry states."""

    def __init__(self) -> None:
        """Configure ROS interfaces and planning parameters."""
        super().__init__('trajectory_planner')

        # Planner tuning parameters.
        self.declare_parameter('current_odom_topic', '/odometry/filtered')
        self.declare_parameter('target_odom_topic', '/planning/target_odom')
        self.declare_parameter('controller_target_topic', '/desired_state')

        # TODO might want these to be more coarse?
        self.declare_parameter('plan_duration_sec', 5.0) # Minimum time to execute any planned trajectory, regardless of distance.
        self.declare_parameter('sample_period_sec', 0.1) # Time between adjacent trajectory setpoints; also affects how long the planner takes to run.
        self.declare_parameter('desired_linear_speed_mps', 0.3)
        self.declare_parameter('dispatch_rate_hz', 10.0)
        self.declare_parameter('position_tolerance_m', 0.10)
        self.declare_parameter('orientation_tolerance_rad', 0.15)
        self.declare_parameter('linear_velocity_tolerance_mps', 0.10)
        self.declare_parameter('angular_velocity_tolerance_radps', 0.20)
        self.declare_parameter('use_target_linear_velocity', True)
        self.declare_parameter('frame_id_fallback', 'odom')

        current_topic = self.get_parameter('current_odom_topic').value
        target_topic = self.get_parameter('target_odom_topic').value
        controller_target_topic = self.get_parameter('controller_target_topic').value
        dispatch_rate_hz = float(self.get_parameter('dispatch_rate_hz').value)
        dispatch_rate_hz = max(dispatch_rate_hz, 1e-3)

        self._current_odom: Odometry | None = None
        self._trajectory_queue: Deque[Odometry] = deque()
        self._active_setpoint: Odometry | None = None

        self._current_sub = self.create_subscription(
            Odometry,
            current_topic,
            self._handle_current_odom,
            10,
        )
        self._target_sub = self.create_subscription(
            Odometry,
            target_topic,
            self._handle_target_odom,
            10,
        )

        self._point_pub = self.create_publisher(Odometry, controller_target_topic, 10)
        self._dispatch_timer = self.create_timer(1.0 / dispatch_rate_hz, self._dispatch_setpoint)

        self.get_logger().info(
            f'trajectory_planner ready: current={current_topic}, target={target_topic}, '
            f'controller_target={controller_target_topic}'
        )

    # Input callbacks.
    def _handle_current_odom(self, msg: Odometry) -> None:
        """Store the latest current vehicle state."""
        self.get_logger().info('Received current odometry update.')
        self._current_odom = msg

    def _handle_target_odom(self, target_odom: Odometry) -> None:
        """Replan queue whenever a mission target is received."""
        if self._current_odom is None:
            self.get_logger().warning('Received target odometry before current state; ignoring.')
            return

        trajectory_points = self._plan_trajectory(self._current_odom, target_odom)
        if not trajectory_points:
            self.get_logger().warning('Planner produced no points; trajectory not published.')
            return

        # Replace any active trajectory with a newly planned queue.
        self._trajectory_queue = deque(trajectory_points)
        self._active_setpoint = None

        self._publish_next_setpoint()

        self.get_logger().info(
            f'Planned trajectory with {len(trajectory_points)} sampled points '
            f'over {self._compute_duration(self._current_odom, target_odom):.2f}s.'
        )

    def _dispatch_setpoint(self) -> None:
        """Advance the setpoint queue when the current setpoint has been reached."""
        self.get_logger().info(f'Current target setpoint: {self._active_setpoint}')
        self.get_logger().info(f'Current vehicle state: {self._current_odom}')
        if self._current_odom is None or self._active_setpoint is None:
            return

        if not self._is_setpoint_reached(self._current_odom, self._active_setpoint):
            return

        self._publish_next_setpoint()

    def _publish_next_setpoint(self) -> None:
        """Pop the next queued target point and publish it to the controller."""
        if not self._trajectory_queue:
            self._active_setpoint = None
            return

        self._active_setpoint = self._trajectory_queue.popleft()
        setpoint_msg = copy.deepcopy(self._active_setpoint)
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        self._point_pub.publish(setpoint_msg)

    # Trajectory planning.
    def _plan_trajectory(self, current_odom: Odometry, target_odom: Odometry) -> List[Odometry]:
        """Generate smooth intermediate odometry states from current to target."""
        sample_period = float(self.get_parameter('sample_period_sec').value)
        sample_period = max(sample_period, 1e-3)

        duration = self._compute_duration(current_odom, target_odom)
        num_samples = max(2, int(math.floor(duration / sample_period)) + 1)

        p0 = _vector3_to_numpy(current_odom.pose.pose.position)
        p1 = _vector3_to_numpy(target_odom.pose.pose.position)

        v0 = _vector3_to_numpy(current_odom.twist.twist.linear)
        use_target_linear_velocity = bool(self.get_parameter('use_target_linear_velocity').value)
        v1 = _vector3_to_numpy(target_odom.twist.twist.linear) if use_target_linear_velocity else np.zeros(3)

        a0 = np.zeros(3)
        a1 = np.zeros(3)

        coeffs = np.zeros((3, 6), dtype=float)
        for axis in range(3):
            boundary = QuinticBoundary(
                p0=p0[axis],
                v0=v0[axis],
                a0=a0[axis],
                p1=p1[axis],
                v1=v1[axis],
                a1=a1[axis],
            )
            coeffs[axis, :] = _solve_quintic(boundary, duration)

        q0 = _quat_xyzw_to_numpy(current_odom.pose.pose.orientation)
        q1 = _quat_xyzw_to_numpy(target_odom.pose.pose.orientation)
        angular_velocity = _compute_constant_angular_velocity(q0, q1, duration)

        frame_id = (
            target_odom.header.frame_id
            or current_odom.header.frame_id
            or str(self.get_parameter('frame_id_fallback').value)
        )
        child_frame_id = target_odom.child_frame_id or current_odom.child_frame_id

        start_time = self.get_clock().now()

        # Sample trajectory at fixed time intervals and create list of setpoint messages.

        trajectory_points: List[Odometry] = []
        for sample_index in range(num_samples):
            t = min(sample_index * sample_period, duration)
            u = t / duration if duration > 1e-6 else 1.0

            position = np.array([_eval_quintic(coeffs[axis, :], t) for axis in range(3)], dtype=float)
            linear_velocity = np.array(
                [_eval_quintic_first_derivative(coeffs[axis, :], t) for axis in range(3)],
                dtype=float,
            )
            orientation = _slerp_xyzw(q0, q1, u)

            point = Odometry()
            point.header.frame_id = frame_id
            point.header.stamp = (start_time + Duration(seconds=t)).to_msg()
            point.child_frame_id = child_frame_id

            point.pose.pose.position.x = float(position[0])
            point.pose.pose.position.y = float(position[1])
            point.pose.pose.position.z = float(position[2])

            point.pose.pose.orientation.x = float(orientation[0])
            point.pose.pose.orientation.y = float(orientation[1])
            point.pose.pose.orientation.z = float(orientation[2])
            point.pose.pose.orientation.w = float(orientation[3])

            point.twist.twist.linear.x = float(linear_velocity[0])
            point.twist.twist.linear.y = float(linear_velocity[1])
            point.twist.twist.linear.z = float(linear_velocity[2])

            point.twist.twist.angular.x = float(angular_velocity[0])
            point.twist.twist.angular.y = float(angular_velocity[1])
            point.twist.twist.angular.z = float(angular_velocity[2])

            trajectory_points.append(point)

        return trajectory_points

    def _compute_duration(self, current_odom: Odometry, target_odom: Odometry) -> float:
        """Choose a duration that respects both configured minimum time and travel distance."""
        min_duration = float(self.get_parameter('plan_duration_sec').value)
        desired_speed = float(self.get_parameter('desired_linear_speed_mps').value)
        desired_speed = max(desired_speed, 1e-3)

        p0 = _vector3_to_numpy(current_odom.pose.pose.position)
        p1 = _vector3_to_numpy(target_odom.pose.pose.position)
        distance = float(np.linalg.norm(p1 - p0))

        kinematic_duration = distance / desired_speed
        return max(min_duration, kinematic_duration)

    def _is_setpoint_reached(self, current_odom: Odometry, setpoint_odom: Odometry) -> bool:
        """Check whether vehicle state is within pose and velocity tolerances."""
        position_tolerance_m = float(self.get_parameter('position_tolerance_m').value)
        orientation_tolerance_rad = float(self.get_parameter('orientation_tolerance_rad').value)
        linear_velocity_tolerance_mps = float(
            self.get_parameter('linear_velocity_tolerance_mps').value
        )
        angular_velocity_tolerance_radps = float(
            self.get_parameter('angular_velocity_tolerance_radps').value
        )

        position_tolerance_m = max(position_tolerance_m, 0.0)
        orientation_tolerance_rad = max(orientation_tolerance_rad, 0.0)
        linear_velocity_tolerance_mps = max(linear_velocity_tolerance_mps, 0.0)
        angular_velocity_tolerance_radps = max(angular_velocity_tolerance_radps, 0.0)

        # position
        p_current = _vector3_to_numpy(current_odom.pose.pose.position)
        p_setpoint = _vector3_to_numpy(setpoint_odom.pose.pose.position)
        position_error_m = float(np.linalg.norm(p_setpoint - p_current))

        # orientation
        q_current = _quat_xyzw_to_numpy(current_odom.pose.pose.orientation)
        q_setpoint = _quat_xyzw_to_numpy(setpoint_odom.pose.pose.orientation)
        orientation_error_rad = _quaternion_angular_distance_rad(q_current, q_setpoint)

        # linear velocity
        v_current = _vector3_to_numpy(current_odom.twist.twist.linear)
        v_setpoint = _vector3_to_numpy(setpoint_odom.twist.twist.linear)
        linear_velocity_error_mps = float(np.linalg.norm(v_setpoint - v_current))

        # angular velocity
        omega_current = _vector3_to_numpy(current_odom.twist.twist.angular)
        omega_setpoint = _vector3_to_numpy(setpoint_odom.twist.twist.angular)
        angular_velocity_error_radps = float(np.linalg.norm(omega_setpoint - omega_current))

        return (
            position_error_m <= position_tolerance_m
            and orientation_error_rad <= orientation_tolerance_rad
            and linear_velocity_error_mps <= linear_velocity_tolerance_mps
            and angular_velocity_error_radps <= angular_velocity_tolerance_radps
        )


################## HELPER FUNCTIONS #############################
# Quintic helpers.
def _solve_quintic(boundary: QuinticBoundary, duration: float) -> np.ndarray:
    """Solve quintic polynomial coefficients for boundary position/velocity/acceleration."""
    # solving for coefficients of a quintic polynomial:
    # p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    # subject to boundary conditions on p, v, a at t=0 and t=duration.
    t = max(duration, 1e-6)

    system = np.array(
        [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
            [1.0, t, t**2, t**3, t**4, t**5],
            [0.0, 1.0, 2.0 * t, 3.0 * t**2, 4.0 * t**3, 5.0 * t**4],
            [0.0, 0.0, 2.0, 6.0 * t, 12.0 * t**2, 20.0 * t**3],
        ],
        dtype=float,
    )

    values = np.array(
        [boundary.p0, boundary.v0, boundary.a0, boundary.p1, boundary.v1, boundary.a1],
        dtype=float,
    )

    return np.linalg.solve(system, values)


def _eval_quintic(coefficients: np.ndarray, t: float) -> float:
    """Evaluate quintic position at time `t`."""
    a0, a1, a2, a3, a4, a5 = coefficients
    return float(a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5)


def _eval_quintic_first_derivative(coefficients: np.ndarray, t: float) -> float:
    """Evaluate quintic velocity at time `t`."""
    _, a1, a2, a3, a4, a5 = coefficients
    return float(a1 + 2.0 * a2 * t + 3.0 * a3 * t**2 + 4.0 * a4 * t**3 + 5.0 * a5 * t**4)


# Quaternion helpers.
def _quat_xyzw_to_numpy(quaternion) -> np.ndarray:
    """Convert ROS xyzw quaternion to normalized numpy array."""
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def _quat_conjugate_xyzw(q: np.ndarray) -> np.ndarray:
    """Compute quaternion conjugate for xyzw quaternion representation."""
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_multiply_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions represented as xyzw arrays."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def _slerp_xyzw(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    """Spherical linear interpolation for normalized xyzw quaternions."""
    q_start = q0.copy()
    q_end = q1.copy()

    dot = float(np.dot(q_start, q_end))
    if dot < 0.0:
        q_end = -q_end
        dot = -dot

    dot = float(np.clip(dot, -1.0, 1.0))

    if dot > 0.9995:
        q = q_start + u * (q_end - q_start)
        return q / np.linalg.norm(q)

    theta = math.acos(dot)
    sin_theta = math.sin(theta)

    scale_0 = math.sin((1.0 - u) * theta) / sin_theta
    scale_1 = math.sin(u * theta) / sin_theta

    q = scale_0 * q_start + scale_1 * q_end
    return q / np.linalg.norm(q)


def _compute_constant_angular_velocity(q0: np.ndarray, q1: np.ndarray, duration: float) -> np.ndarray:
    """Estimate constant angular velocity that rotates q0 to q1 over `duration`."""
    t = max(duration, 1e-6)

    q_start = q0.copy()
    q_goal = q1.copy()
    if float(np.dot(q_start, q_goal)) < 0.0:
        q_goal = -q_goal

    q_rel = _quat_multiply_xyzw(_quat_conjugate_xyzw(q_start), q_goal)
    q_rel = q_rel / np.linalg.norm(q_rel)

    w = float(np.clip(q_rel[3], -1.0, 1.0))
    angle = 2.0 * math.acos(w)

    sin_half_angle = math.sqrt(max(1.0 - w * w, 0.0))
    if sin_half_angle < 1e-6 or angle < 1e-6:
        return np.zeros(3, dtype=float)

    axis = q_rel[:3] / sin_half_angle
    return axis * (angle / t)


def _quaternion_angular_distance_rad(q0: np.ndarray, q1: np.ndarray) -> float:
    """Return the shortest angular distance between two orientations in radians."""
    q_start = q0.copy()
    q_goal = q1.copy()
    if float(np.dot(q_start, q_goal)) < 0.0:
        q_goal = -q_goal

    q_rel = _quat_multiply_xyzw(_quat_conjugate_xyzw(q_start), q_goal)
    q_rel = q_rel / np.linalg.norm(q_rel)
    w = float(np.clip(q_rel[3], -1.0, 1.0))
    return 2.0 * math.acos(w)


def _vector3_to_numpy(vector3) -> np.ndarray:
    """Convert ROS Vector3/Point-compatible object to numpy xyz vector."""
    return np.array([vector3.x, vector3.y, vector3.z], dtype=float)


def main(args=None) -> None:
    """Run the trajectory planner node."""
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
