"""
Boustrophedon (lawnmower) baseline survey.

A standalone action client: it computes an exhaustive lawnmower coverage path
over a rectangular ROI in-file, then dispatches the waypoints one at a time as
PoseToPose goals to the Dubins action server (vista_sim). No behavior tree, no
sampling server. This is the non-adaptive baseline against which the NBV
mission (helix / cone) is compared.

Frame convention (matches dubins_pose_to_pose_action_server.py):
    NED. position.x = north, position.y = east, position.z = depth (down +).
    yaw: 0 = heading north (+x), pi = heading south (-x).

The vehicle spawns at Eta(-10, 0, 0, yaw=0): north=-10, east=0, facing north.
That sits just south of the first lane (which starts at north=-row_extension,
≈-7.58 m at clearance=15), so the first row is a clean northward sweep with a
built-in running start instead of a U-turn loop. Both this baseline and the NBV
mission share that one hardcoded spawn, so no spawn parameter is needed.

Geometry
--------
The forward-looking sonar sees a footprint on the seafloor. With the sensor at
clearance `z` above the floor and a horizontal field of view HFOV, the
cross-track swath is governed purely by the FOV (the mount angle does NOT enter
the swath):

    swath_cross  = 2 * z * tan(HFOV / 2)
    lane_spacing = swath_cross / 2            # 50% side overlap

The mount angle DOES set the along-track "running start": the oblique axis
throws the footprint forward, so each row is extended past its endpoints by the
along-track footprint so the swath fully sweeps the row ends before the vehicle
arrives / after it departs:

    row_extension = z * (tan(mount + VFOV/2) - tan(mount - VFOV/2))
"""

import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from uuv_interfaces.action import PoseToPose


class BoustrophedonRun(Node):
    """Compute a lawnmower path and dispatch it as sequential PoseToPose goals."""

    def __init__(self):
        super().__init__("boustrophedon_run")

        # ---- Tunable mission parameters (edit here) -------------------------
        # Sensor field of view (forward-looking sonar), degrees.
        self.fov_x_deg = 150.0          # horizontal (cross-track) FOV
        self.fov_y_deg = 25.0           # vertical (along-track) FOV

        # Sensor mounting depression from nadir, degrees. Sets the running start
        # only; it does NOT affect the swath / lane spacing.
        self.mount_angle_deg = 20.0

        # Sensor clearance above the seafloor, metres. Drives the swath geometry.
        self.clearance = 15.0

        # ROI rectangle to cover, NED metres. Rows run along north; lanes step
        # along east. Matches bayesian_search's 1000x1000 arena (origin 0,0) so
        # the baseline and the search cover an identical region.
        self.roi_north_min = 0.0
        self.roi_north_max = 1000.0
        self.roi_east_min = 0.0
        self.roi_east_max = 1000.0

        # Pose frame; must match the action server's frame_id.
        self.frame_id = "ned"
        # ---------------------------------------------------------------------

        # Survey depth is DERIVED, not set: resolved from the ned<-map TF so the
        # vehicle flies exactly `clearance` metres above the seafloor.
        #   survey_depth = seafloor_depth - clearance
        # (bayesian_search fixes search_depth and derives clearance; we invert.)
        self.seafloor_depth = None
        self.survey_depth = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._client = ActionClient(self, PoseToPose, "pose_to_pose")

    # ---- Geometry -----------------------------------------------------------
    def resolve_survey_depth(self):
        """Resolve survey_depth from the ned<-map TF: clearance above the floor.

        seafloor_depth is the NED depth of the map origin (seafloor at map z=0),
        i.e. the z translation of ned<-map. Returns True once resolved, False if
        the (static) TF is not available yet.
        """
        if self.survey_depth is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform('ned', 'map', rclpy.time.Time())
        except tf2_ros.TransformException:
            return False  # TF not ready yet

        self.seafloor_depth = tf.transform.translation.z
        self.survey_depth = self.seafloor_depth - self.clearance
        self.get_logger().info(
            f"Resolved survey_depth={self.survey_depth:.2f}m "
            f"(seafloor_depth={self.seafloor_depth:.2f}m - "
            f"clearance={self.clearance:.2f}m)"
        )
        return True

    def compute_geometry(self):
        """Derive swath, lane spacing, and row extension from the FOV / mount."""
        hfov = math.radians(self.fov_x_deg)
        vfov = math.radians(self.fov_y_deg)
        mount = math.radians(self.mount_angle_deg)
        z = self.clearance

        self.swath_cross = 2.0 * z * math.tan(hfov / 2.0)
        self.lane_spacing = self.swath_cross / 2.0      # 50% overlap
        self.row_extension = z * (math.tan(mount + vfov / 2.0)
                                  - math.tan(mount - vfov / 2.0))

        self.get_logger().info(
            f"Geometry @ z={z:.1f}m: swath_cross={self.swath_cross:.2f}m, "
            f"lane_spacing={self.lane_spacing:.2f}m, "
            f"row_extension={self.row_extension:.2f}m"
        )

    def compute_lawnmower_poses(self):
        """Build the boustrophedon waypoint list (row endpoints, alternating)."""
        self.compute_geometry()

        # Lane centres across east; step by lane_spacing, inclusive of east_max.
        lanes = np.arange(self.roi_east_min,
                          self.roi_east_max + self.lane_spacing,
                          self.lane_spacing)

        north_lo = self.roi_north_min - self.row_extension
        north_hi = self.roi_north_max + self.row_extension

        poses = []
        for i, east in enumerate(lanes):
            if i % 2 == 0:
                start_n, end_n, yaw = north_lo, north_hi, 0.0       # heading north
            else:
                start_n, end_n, yaw = north_hi, north_lo, math.pi   # heading south
            poses.append(self._make_pose(start_n, east, yaw))
            poses.append(self._make_pose(end_n, east, yaw))

        self.get_logger().info(
            f"Lawnmower: {len(lanes)} lanes, {len(poses)} waypoints."
        )
        return poses

    def _make_pose(self, north, east, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(north)
        pose.pose.position.y = float(east)
        pose.pose.position.z = float(self.survey_depth)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # ---- Dispatch -----------------------------------------------------------
    def run(self):
        # Resolve survey depth from the ned<-map TF before building poses, so the
        # vehicle flies `clearance` above the seafloor (the static TF is latched).
        self.get_logger().info("Waiting for ned<-map TF to resolve survey depth...")
        while rclpy.ok() and not self.resolve_survey_depth():
            rclpy.spin_once(self, timeout_sec=0.1)

        poses = self.compute_lawnmower_poses()

        self.get_logger().info("Waiting for pose_to_pose action server...")
        self._client.wait_for_server()

        for idx, pose in enumerate(poses):
            self.get_logger().info(
                f"Dispatching waypoint {idx + 1}/{len(poses)}: "
                f"N={pose.pose.position.x:.1f} E={pose.pose.position.y:.1f}"
            )
            if not self._send_and_wait(pose):
                self.get_logger().error(
                    f"Waypoint {idx + 1} failed; aborting survey."
                )
                return
        self.get_logger().info("Boustrophedon survey complete.")

    def _send_and_wait(self, pose):
        """Send one goal and block (spinning) until its result returns."""
        goal = PoseToPose.Goal()
        goal.goal_pose = pose

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by server.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"  -> {result.result_message} "
            f"(dist={result.distance_to_goal:.2f}m, "
            f"yaw_err={result.yaw_error_at_goal:.3f}rad)"
        )
        # Both "Succeeded" and "Missed" are server successes (goal_handle.succeed()):
        # a "Missed" just means the vehicle flew the whole lane and overshot the
        # endpoint past the threshold, which is fine for coverage. Only "Canceled"
        # / "Aborted" are real failures.
        return result.result_message in ("Succeeded", "Missed")


def main(args=None):
    rclpy.init(args=args)
    node = BoustrophedonRun()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()