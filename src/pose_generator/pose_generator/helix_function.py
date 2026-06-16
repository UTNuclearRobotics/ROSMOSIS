import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation




def generate_helix(
    r_min: float,
    radius_multiplier: float,
    num_shells: int,
    helix_height: float,
    clearance: float,
    psi_max_deg: float,
    delta_theta_deg: float,
    mount_angle_deg: float,
) -> PoseArray:
    """
    Generate candidate viewpoint poses on a helical manifold around an ROI center.

    For each cylindrical shell at radius r_i, a helix is traced from the top
    (clearance + helix_height) downward using a linear pitch z = (h_max/2pi)*theta.
    Poses represent the desired sonar_optical frame in the ROI-local frame,
    with heading tangent to the helix and the FLS mount angle applied.

    Args:
        r_min:             Minimum standoff radius in metres (= Dubins turn radius)
        radius_multiplier: Max radius = radius_multiplier * r_min
        num_shells:        Number of concentric cylindrical shells
        helix_height:      Vertical extent of helix above clearance in metres
        clearance:         Minimum height above seabed in metres
        psi_max_deg:       Maximum vehicle climb angle in degrees
        delta_theta_deg:   Angular step between samples in degrees
        mount_angle_deg:   FLS downward mount angle from horizontal in degrees

    Returns:
        PoseArray with header.frame_id='roi_frame', one Pose per sample point
    """
    
    # necessary deg 2 rad conversions
    delta_theta = np.radians(delta_theta_deg)
    mount_angle = np.radians(mount_angle_deg)
    psi_max = np.radians(psi_max_deg)
    
    
    
    
    r_max = r_min * radius_multiplier
    radii = np.linspace(r_min, r_max, num=num_shells, endpoint=True, dtype=float)

    pose_array = PoseArray()
    pose_array.header.frame_id = 'roi_frame'
    
    # note that the theta list (linspace) depends on the pitch, which depends on r
    # as the radius increases, the total theta decreases. 
    # for constant theta increment, this decreasing the num samples on each helix.
    # essentially:
    # radius -> changes pitch -> changes total theta -> changes num samples on each helix for const del_theta
    
    for r in radii:
        # compute maximum allowable pitch h (height climb for one revolution spiral)
        h_max = 2*np.pi * r * np.tan(psi_max)
        # compute total theta over all spirals.
        theta_total = helix_height * (2*np.pi)/(h_max)
        num_samples = int(np.ceil(theta_total/delta_theta))
        
        #compute theta linspace
        
        theta_vec = np.linspace(0, theta_total, num = num_samples, endpoint= True, dtype = float)
        
        # now create starting pose, which we refer to as having R_0 and t_0
        # to show parameterization, we will index theta_vec
        
        theta_0 = theta_vec[0]
        t_0= np.array([r*np.cos(theta_0), r*np.sin(theta_0), helix_height + clearance -(h_max/(2*np.pi) * theta_0)]) # 3 x 1

        # Mount-angle offset: shift the pose backward along the pre-mount optical
        # axis (the unit horizontal tangent h_hat) so the down-tilted FLS beam
        # still lands on the radius-r ground circle. offset = z / tan(mount_angle),
        # using the pose's world height (roi base at z=0). Applied before the mount
        # rotation so the shift stays horizontal and the height is preserved.
        h_hat_0 = np.array([-np.sin(theta_0), np.cos(theta_0), 0.0])
        t_0 = t_0 - (t_0[2] / np.tan(mount_angle)) * h_hat_0

        R_0= Rotation.from_matrix(np.array([[1,0,0],[0,0,-1],[0,1,0]]).T)


        #apply body rotation
        R_body = Rotation.from_euler('x', -mount_angle)
        R_0 = R_0 * R_body
        
        
        #before looping, append to PoseArray
        pose0 = Pose()
        
        quat0 = R_0.as_quat()
        #populate translation associated with pose object
        pose0.position.x, pose0.position.y, pose0.position.z = t_0[0], t_0[1], t_0[2]
        
        #populate orientation of pose object
        pose0.orientation.x = quat0[0]
        pose0.orientation.y = quat0[1]
        pose0.orientation.z = quat0[2]
        pose0.orientation.w = quat0[3]
        
        pose_array.poses.append(pose0)
        
        for theta in theta_vec[1:]:
            pose_i = Pose()
            t_i= np.array([r*np.cos(theta), r*np.sin(theta), helix_height + clearance -(h_max/(2*np.pi) * theta)]) # 3 x 1

            # Same mount-angle offset as the seed pose: backward along the unit
            # horizontal tangent by z / tan(mount_angle), before the mount rotation.
            h_hat = np.array([-np.sin(theta), np.cos(theta), 0.0])
            t_i = t_i - (t_i[2] / np.tan(mount_angle)) * h_hat

            R_theta = Rotation.from_euler('z', theta)
            
            R_i = R_theta * R_0
            quat_i = R_i.as_quat()
            
            #populate pose
            pose_i.position.x, pose_i.position.y, pose_i.position.z = t_i[0], t_i[1], t_i[2]
            
            pose_i.orientation.x = quat_i[0]
            pose_i.orientation.y = quat_i[1]
            pose_i.orientation.z = quat_i[2]
            pose_i.orientation.w = quat_i[3]
        
            pose_array.poses.append(pose_i)

    return pose_array


if __name__ == "__main__":
    # Test with params from nbv_on_target.xml
    pose_array = generate_helix(
        r_min=5.0,
        radius_multiplier=5.0,
        num_shells=4,
        helix_height=10.0,
        clearance=5.0,
        psi_max_deg=15.0,
        delta_theta_deg=30.0,
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

    # Plot helix trajectory
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
    ax.set_title(f'Helix Viewpoints ({len(pose_array.poses)} poses) with Sensor Frames')
    ax.legend()

    plt.show()
            
            
            

            
            
            
            
            
            
            
            
            

            
            
        
    
        
        