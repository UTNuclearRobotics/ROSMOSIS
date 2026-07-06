"""
Author: Talal Ayman
ROS 2 Action Server that drives a simple Dubins-based vehicle simulator
from its current pose to a goal pose.

Key features
------------
*   No changes to the Dubins planner or vehicle dynamics classes.
*   TF broadcast of `base_link` for RViz visualisation.
*   Blocking execute callback running in a MultiThreadedExecutor thread.
*   Client-side cancellation support with MutuallyExclusiveCallbackGroup.
*   Drift node integration planned (pause/resume service calls).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from uuv_interfaces.action import PoseToPose

from vista_sim.simple_vehicle_sim_v3 import (
    Eta,
    Nu,
    DubinsAirplanePath,
    SimpleVehicleModel,
)


class VehiclePoseActionServer(Node):
    """Pose-to-Pose action server using Dubins planning and a blocking execute callback."""

    def __init__(self):
        super().__init__("vehicle_pose_action_server")

        # Parameters
        self.declare_parameter("frame_id", "ned")
        self.declare_parameter("time_step", 0.1)
        self.declare_parameter("constant_velocity", 1.5)
        self.declare_parameter("drift_velocity", 0.25)
        self.declare_parameter("turn_radius_m", 10.0)
        self.declare_parameter("max_pitch_deg", 15.0)
        self.frame_id = self.get_parameter("frame_id").value
        self.dt = self.get_parameter("time_step").value
        self.constant_velocity = self.get_parameter("constant_velocity").value
        self._drift_velocity = self.get_parameter("drift_velocity").value
        
        # Vehicle's minimum turn radius and max pitch.
        self.turn_radius_m = self.get_parameter("turn_radius_m").value
        self.max_pitch_deg = self.get_parameter("max_pitch_deg").value

        # Vehicle state — always valid, never None. This node is the SOLE owner
        # of the ned->base_link transform. execute_cb broadcasts during a goal;
        # the idle-drift timer broadcasts between goals. _goal_active gates the
        # two so they never publish concurrently — there is no second node and
        # no pause/resume RPC to desync (that desync was the 20 Hz dual-publish
        # that produced the flicker).
        
        # initializing eta also determines its initial position relative to ned.
        # North=-30: south of the boustrophedon's first lane (north = -row_extension
        # = -7.58 m at clearance=15), with ~22 m of run-up so the vehicle reaches
        # cruise speed (1.5 m/s at 1.0 m/s^2 accel) and settles well before the
        # first sweep lane, instead of a wasteful U-turn loop at turn_radius=10.
        self._eta = Eta(-30.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._nu = Nu(self._drift_velocity, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._goal_active = False

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self._broadcast_initial()

        # Action Server with cancellation support
        motion_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self,
            PoseToPose,
            "pose_to_pose",
            goal_callback=self.goal_cb,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            callback_group=motion_cb_group,
        )

        # Idle-drift timer — propagates the vehicle at idle drift velocity and
        # broadcasts base_link when no goal is executing. Gated by _goal_active
        # so it stays silent during navigation (execute_cb owns the broadcast
        # then). Replaces the former external drift_service + pause/resume RPCs.
        self.create_timer(self.dt, self._idle_drift_cb)

        self.get_logger().info("Vehicle Pose Action Server ready")

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def goal_cb(self, goal_request):
        if not isinstance(goal_request.goal_pose, PoseStamped):
            self.get_logger().warn("Goal rejected: goal_pose is not a PoseStamped.")
            return GoalResponse.REJECT
        self.get_logger().info("Goal received – accepting.")
        return GoalResponse.ACCEPT

    def cancel_cb(self, cancel_request):
        # Just accept; cancellation is tracked per-goal via goal_handle.is_cancel_requested
        # (set by rclpy's action machinery, independent of this callback group's timing).
        self.get_logger().info("Cancel accepted – halting navigation.")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Coordinate conversion helpers
    # ------------------------------------------------------------------

    def pose_stamped_to_eta(self, pose: PoseStamped) -> Eta:
        """Convert PoseStamped to Eta."""
        qx = pose.pose.orientation.x
        qy = pose.pose.orientation.y
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return Eta(
            north=pose.pose.position.x,
            east=pose.pose.position.y,
            depth=pose.pose.position.z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def eta_to_pose_stamped(self, eta: Eta) -> PoseStamped:
        """Convert Eta to PoseStamped."""
        x, y, z, w = quaternion_from_euler(eta.roll, eta.pitch, eta.yaw)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = eta.north
        pose.pose.position.y = eta.east
        pose.pose.position.z = eta.depth
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def broadcast_tf(self, pose: PoseStamped):
        """Broadcast base_link TF for RViz visualisation."""
        t = TransformStamped()
        t.header.stamp = pose.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = "base_link"
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _broadcast_initial(self):
        """Broadcast the initial ned -> base_link transform derived from _eta."""
        self.broadcast_tf(self.eta_to_pose_stamped(self._eta))

    def _idle_drift_cb(self):
        """Propagate idle drift and broadcast base_link between goals.

        Gated by _goal_active: while a goal is executing, execute_cb owns the
        base_link broadcast and this returns immediately, so the two never
        publish the transform concurrently. With no second node and no RPC,
        there is nothing left to desync — base_link stays a single 10 Hz stream.
        """
        if self._goal_active:
            return
        eta = self._eta
        self._eta = Eta(
            north=eta.north + self._drift_velocity * math.cos(eta.yaw) * self.dt,
            east=eta.east + self._drift_velocity * math.sin(eta.yaw) * self.dt,
            depth=eta.depth,
            roll=0.0,
            pitch=0.0,
            yaw=eta.yaw,
        )
        self._nu = Nu(self._drift_velocity, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.broadcast_tf(self.eta_to_pose_stamped(self._eta))
    # ------------------------------------------------------------------
    # Execute callback – runs in its own thread (MultiThreadedExecutor)
    # ------------------------------------------------------------------

    def execute_cb(self, goal_handle):
        
        """Blocking execution loop: plan once, then step the vehicle model."""
        self.get_logger().info("Execution started.")

        # Take ownership of the base_link broadcast for this goal; the idle-
        # drift timer stays silent while this is True.
        self._goal_active = True

        goal_eta = self.pose_stamped_to_eta(goal_handle.request.goal_pose)

        # Seed from the node's current (idle-drift) pose instead of an RPC.
        eta, nu = self._eta, self._nu


        # --- Plan path once ---
        # Plan 20% looser than the vehicle's minimum turn radius so tracking has
        # yaw-rate headroom to correct cross-track error instead of saturating
        # (mirrors simple_vehicle_sim_v3.py).
        planner = DubinsAirplanePath(
            turn_radius=1.2 * self.turn_radius_m, max_pitch_deg=self.max_pitch_deg
        )
        waypoints = planner.get_poses(eta, goal_eta, waypoint_spacing=1.0)
        self.get_logger().info(f"Planned Dubins path with {len(waypoints)} waypoints.")

        vehicle = SimpleVehicleModel(
            {
                "speed_time_constant": 2.0,
                "yaw_time_constant": 0.2,
                "pitch_time_constant": 1.5,
                "roll_ratio": 0.2,
                "turn_radius_m": self.turn_radius_m,
                "max_acceleration_mps2": 1.0,
                "max_speed_mps": self.constant_velocity,
                "max_pitch_deg": self.max_pitch_deg,
                "pitch_proportional_gain": 0.5,
                "look_ahead_dist": 1.0,
                "nominal_speed": self.constant_velocity,
            },
            waypoints,
        )

        result = PoseToPose.Result()
        rate = self.create_rate(1.0 / self.dt)
        min_dist = float("inf")  # closest approach seen so far

        # --- Follow path ---
        while True:
            if goal_handle.is_cancel_requested:
                self._eta, self._nu = eta, nu
                self._goal_active = False  # hand base_link back to idle drift
                goal_handle.canceled()
                self.get_logger().info("Goal canceled.")
                result.result_message = "Canceled"
                result.distance_to_goal = -1.0
                result.yaw_error_at_goal = -1.0
                return result

            control = vehicle.calc_control_input(eta)
            eta, nu = vehicle.step(eta, nu, control, self.dt)

            # Update shared state and broadcast TF
            self._eta, self._nu = eta, nu
            pose_msg = self.eta_to_pose_stamped(eta)
            self.broadcast_tf(pose_msg)

            dist = math.sqrt(
                (eta.north - goal_eta.north) ** 2
                + (eta.east - goal_eta.east) ** 2
                + (eta.depth - goal_eta.depth) ** 2
            )
            min_dist = min(min_dist, dist)

            # yaw_err: shortest angular distance between current heading and goal heading.
            # atan2(sin(Δ), cos(Δ)) wraps the difference safely into [-π, π].
            yaw_err = abs(math.atan2(
                math.sin(eta.yaw - goal_eta.yaw),
                math.cos(eta.yaw - goal_eta.yaw),
            ))

            feedback = PoseToPose.Feedback()
            feedback.current_pose = pose_msg
            feedback.distance_remaining = dist
            feedback.yaw_error_remaining = yaw_err
            goal_handle.publish_feedback(feedback)

            # Nominal success: within position and heading thresholds.
            reached = dist < 3.0 and yaw_err < 0.3

            # Miss detection: path fully consumed (closest waypoint is the last one) and
            # the vehicle is receding from its closest approach. Using path position rather
            # than a geometric bearing check avoids false triggers on Dubins arcs, where
            # the vehicle legitimately heads away from the goal mid-path before curving back.
            pos = np.array([eta.north, eta.east])
            closest_idx = int(np.argmin(np.sum((vehicle.path[:, :2] - pos) ** 2, axis=1)))
            at_path_end = closest_idx == len(vehicle.path) - 1
            missed = at_path_end and dist > min_dist + 0.25
            self.get_logger().debug(
                f"wp={closest_idx}/{len(vehicle.path)-1}  dist={dist:.2f}  min={min_dist:.2f}  end={at_path_end}"
            )

            if reached or missed:
                self._eta, self._nu = eta, nu
                self._goal_active = False  # hand base_link back to idle drift
                goal_handle.succeed()
                if reached:
                    self.get_logger().info(
                        f"Goal reached. pos_err={dist:.3f}m  yaw_err={math.degrees(yaw_err):.2f}deg"
                    )
                else:
                    self.get_logger().warn(
                        f"Goal declared done (miss detected). "
                        f"dist={dist:.3f}m  min_dist={min_dist:.3f}m"
                    )
                result.result_message = "Succeeded" if reached else "Missed"
                result.distance_to_goal = dist
                result.yaw_error_at_goal = yaw_err
                return result

            rate.sleep()

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehiclePoseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
