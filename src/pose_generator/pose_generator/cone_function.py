import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation




def generate_cone(
    num_rings: int,
    clearance: float,
    cone_height: float,
    delta_theta_deg: float,
    mount_angle_deg: float,
) -> PoseArray:
    """
    Generate candidate viewpoint poses on an apex-at-origin conical manifold
    around an ROI center.

    The cone's apex sits at the ROI origin: at height z, the standoff radius is
    r = z / tan(mount_angle). Because that radius is derived from the FLS mount
    angle, the down-tilted optical axis lands on the ROI origin BY CONSTRUCTION,
    so (unlike the helix) no tangent-offset shift is applied.

    Rings are placed at num_rings uniform heights in [clearance, cone_height].
    On each ring, samples are spaced every delta_theta_deg of azimuth. Poses
    represent the desired sonar_optical frame in the ROI-local frame, with a
    radial-inward heading and the fixed FLS mount angle applied.

    Unlike the helix, pitch (psi_max) is NOT a parameter here: it does not shape
    this manifold, it only gates reachability between rings, which is handled by
    the planner (Dubins cost server), not the sampler.

    Args:
        num_rings:       Number of concentric rings (z-levels)
        clearance:       Minimum height above seabed in metres (z lower bound)
        cone_height:     Maximum height in metres (z upper bound)
        delta_theta_deg: Angular step between samples in degrees
        mount_angle_deg: FLS downward mount angle from horizontal in degrees
                         (cone half-angle; r = z / tan(mount_angle))

    Returns:
        PoseArray with header.frame_id='roi_frame', one Pose per sample point
    """

    # necessary deg 2 rad conversions
    delta_theta = np.radians(delta_theta_deg)
    mount_angle = np.radians(mount_angle_deg)

    # z-levels: uniform rings between clearance and cone_height
    z_levels = np.linspace(clearance, cone_height, num=num_rings, endpoint=True, dtype=float)

    # azimuth samples (shared across rings, since r is height-derived not theta-driven)
    num_samples = int(np.ceil((2 * np.pi) / delta_theta))
    theta_vec = np.linspace(0, 2 * np.pi, num=num_samples, endpoint=False, dtype=float)

    pose_array = PoseArray()
    pose_array.header.frame_id = 'roi_frame'

    # R_0: pre-azimuth seed orientation. theta=0 places the pose at [0, -r, z]
    # (directly in front of the ROI in ENU, -Y side); the seed boresight is +Y,
    # i.e. radial-INWARD toward the origin. Same seed matrix as the helix; the
    # radial (vs tangential) heading comes from the position parameterization
    # p = [r*sin(theta), -r*cos(theta), z] paired with the R_z(theta) sweep.
    # .T accounts for scipy treating the rows-as-written as the basis columns.
    R_0 = Rotation.from_matrix(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]).T)

    # fixed FLS mount tilt about the body x-axis
    R_body = Rotation.from_euler('x', -mount_angle)
    R_0 = R_0 * R_body

    for z in z_levels:
        # radius derived from height via the cone half-angle (apex at origin)
        r = z / np.tan(mount_angle)

        for theta in theta_vec:
            pose_i = Pose()
            # position on the ring; no tangent offset (r = z/tan(mount) already
            # puts the down-tilted optical axis through the origin). theta=0 is
            # the -Y point (just in front of the ROI in ENU).
            t_i = np.array([r * np.sin(theta), -r * np.cos(theta), z])

            R_theta = Rotation.from_euler('z', theta)
            R_i = R_theta * R_0
            quat_i = R_i.as_quat()

            pose_i.position.x, pose_i.position.y, pose_i.position.z = t_i[0], t_i[1], t_i[2]

            pose_i.orientation.x = quat_i[0]
            pose_i.orientation.y = quat_i[1]
            pose_i.orientation.z = quat_i[2]
            pose_i.orientation.w = quat_i[3]

            pose_array.poses.append(pose_i)

    return pose_array


if __name__ == "__main__":
    # Test with cone-equivalent params (mount_angle/clearance shared with helix)
    pose_array = generate_cone(
        num_rings=3,
        clearance=15.0,
        cone_height=30.0,
        delta_theta_deg=36.0,
        mount_angle_deg=20.0
    )

    # Extract positions for plotting
    positions = np.array([
        [pose.position.x, pose.position.y, pose.position.z]
        for pose in pose_array.poses
    ])

    # Plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot cone samples
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='blue', marker='o', s=20)
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', alpha=0.3)

    # Add insertion order labels
    for idx, pos in enumerate(positions):
        ax.text(pos[0], pos[1], pos[2], str(idx), fontsize=8, color='gray')

    # Plot ENU frame at origin
    axis_length = 1.5
    ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', arrow_length_ratio=0.15, linewidth=2, label='East (X)')
    ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', arrow_length_ratio=0.15, linewidth=2, label='North (Y)')
    ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', arrow_length_ratio=0.15, linewidth=2, label='Up (Z)')

    # Plot local axes at each viewpoint
    frame_scale = 0.5
    for idx, pose in enumerate(pose_array.poses):
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        rot = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        # Get the rotation matrix columns (local x, y, z axes)
        R_mat = rot.as_matrix()
        x_axis = R_mat[:, 0]
        y_axis = R_mat[:, 1]
        z_axis = R_mat[:, 2]

        # Debug: print axes for first pose
        if idx == 0:
            print("\nPlotting axes for pose 0:")
            print(f"  Red (x_axis): {x_axis}")
            print(f"  Green (y_axis): {y_axis}")
            print(f"  Blue (z_axis): {z_axis}")

        # Plot local axes (small, semi-transparent)
        ax.quiver(pos[0], pos[1], pos[2], x_axis[0]*frame_scale, x_axis[1]*frame_scale, x_axis[2]*frame_scale,
                 color='r', alpha=0.3, arrow_length_ratio=0.2, linewidth=0.8)
        ax.quiver(pos[0], pos[1], pos[2], y_axis[0]*frame_scale, y_axis[1]*frame_scale, y_axis[2]*frame_scale,
                 color='g', alpha=0.3, arrow_length_ratio=0.2, linewidth=0.8)
        ax.quiver(pos[0], pos[1], pos[2], z_axis[0]*frame_scale, z_axis[1]*frame_scale, z_axis[2]*frame_scale,
                 color='b', alpha=0.3, arrow_length_ratio=0.2, linewidth=0.8)

    ax.set_xlabel('X (East)')
    ax.set_ylabel('Y (North)')
    ax.set_zlabel('Z (Up)')
    ax.set_title(f'Cone Viewpoints ({len(pose_array.poses)} poses) with Sensor Frames')
    ax.legend()

    plt.show()
