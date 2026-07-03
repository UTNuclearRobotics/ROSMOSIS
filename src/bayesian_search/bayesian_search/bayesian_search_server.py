"""
Bayesian search server for multi-target underwater inspection.

Maintains a probability map (intensity function) over a 2D search region.
The map is updated via two mechanisms:
  1. FLS depth subscription : continuous Bayesian update on observed cells
  2. GetNextSearchPose service request : discrete cluster bump on completed
     targets, with internal delta tracking to avoid double-bumping

Exposes a single service that the BT calls to get the next waypoint to
explore, computed as argmax of utility = P_target / cost over unswept cells.
Cost defaults to 1 in this skeleton; later integrated with a vista_sim
Dubins cost service.
"""

import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
import tf_transformations

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid

from uuv_interfaces.srv import BayesianSearch


class BayesianSearchServer(Node):
    def __init__(self):
        super().__init__('bayesian_search_server')

        # ---- Grid state ----
        self.probability_grid: np.ndarray = None
        self.visited_grid: np.ndarray = None  # boolean swept mask, n x x with visited entry (i,j) set to True

        # ---- Grid metadata ----
        # Arena size is the physical search region (1000x1000m to match the
        # seabed asset). Grid shape is computed from arena_size / resolution.
        self.grid_resolution: float = 1.0     # m per cell
        self.arena_size: tuple = (1000.0, 1000.0) # (width_x, length_y) in m
        self.grid_origin: tuple = (0.0, 0.0)  # (x_min, y_min) in map frame
        
        #in map frame, x corresponds to j index, and y corresponds to i.
        # In NED-specific math (pose generation) this is flipped: tf2 applies the
        # map->ned rotation, so the i axis (map y) aligns with NED north (x) and
        # j (map x) with NED east. The flip is handled by tf2.transform, not by hand.
        self.grid_shape: tuple = (
            int(self.arena_size[1] / self.grid_resolution),
            int(self.arena_size[0] / self.grid_resolution),
        )  # (rows, cols)

        # ---- Configuration / hyperparameters ----
        # Prior peaks at the arena center by default (no edge bias). Override
        # with domain knowledge if targets are expected in a specific region.
        self.prior_centroid: tuple = (
            self.grid_origin[0] + self.arena_size[0] / 2.0,
            self.grid_origin[1] + self.arena_size[1] / 2.0,
        )  # (cx, cy) in map frame
        # prior + cluster bump sigma/amplitude are passed as keyword args to gaussian_bump
        # at the call site (readability), not stored as members.

        # ---- FLS viewing geometry (for the returned search pose) ----
        # In NED (+down): the vehicle holds search_depth; the seafloor depth is
        # resolved lazily from the map->ned TF (set_geometry). clearance is the
        # vehicle's altitude above the seafloor. The forward-tilted FLS means the
        # vehicle sits x_adjustment = clearance / tan(mount_angle) behind (north of)
        # the cell so the centre beam lands on it. Search poses are north-facing.
        self.search_depth: float = 35.0             # NED depth the vehicle holds (m); 50 m seafloor - 15 m clearance
        self.mount_angle: float = np.deg2rad(20.0)  # FLS down-tilt from horizontal
        # Approximate-Dubins cost: distance + turning_radius * |yaw_diff|. The
        # turning_radius (m/rad) converts heading error into equivalent arc length;
        # set it to the Dubins planner's minimum turning radius.
        self.turning_radius: float = 10.0
        self.seafloor_depth: float = None           # resolved by set_geometry via tf
        self.clearance: float = None                # resolved by set_geometry
        self.x_adjustment: float = None             # resolved by set_geometry
        self.T_ned_map: np.ndarray = None           # cached ned<-map 4x4, by set_geometry

        # ---- Internal bookkeeping ----
        # IDs of targets we've already cluster-bumped. Checked each service call
        # so a target seen again (same visited set re-sent) is not bumped twice.
        self.bumped_targets_set: set = set()

        # ---- ROS interfaces ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        ## camera info used for back projection
        self.camera_info = None  # cached once received

        self.depth_sub = self.create_subscription(
            Image, '/sonar/depth/image_raw', self.depth_callback, 10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/sonar/depth/camera_info', self.camera_info_callback,10
        )

        self.belief_pub = self.create_publisher(
            OccupancyGrid, '~/probability_grid', 10
        )

        self.next_pose_srv = self.create_service(
            BayesianSearch, '~/get_next_search_pose', self.get_next_pose_callback
        )

        # Belief-map publishing is RViz-only (see publish_belief) and costs a
        # full-grid normalize + tolist at 2 Hz, wasted on headless/batch runs.
        # Gate the timer on a parameter so those runs skip it entirely. Defaults
        # True, so standalone runs are unchanged; only the timer is gated -- the
        # grid, service, and search logic are untouched either way.
        self.declare_parameter('publish_belief', True)
        if self.get_parameter('publish_belief').value:
            self.belief_timer = self.create_timer(0.5, self.publish_belief)

        self._init_grids()
        self.get_logger().info('BayesianSearchServer ready')

    # ----------------------------------------------------------------------
    # Grid initialization
    # ----------------------------------------------------------------------
    def _init_grids(self):
        """Initialize probability_grid with Gaussian prior and visited_grid as all-False."""
        rows, cols = self.grid_shape
        self.probability_grid =  np.ones((rows, cols)) * 0.01     # baseline floor
        self.visited_grid = np.zeros((rows, cols), dtype=bool)
                
        # need to create a meshgrid over indices i and j, them use this to create the cell_to_map_coords as a member variable to be used
        # cell_to_map_coords: cached (rows, cols, 2) array. so cell_to_map_coords[i,j] = (x_map, y_map)
        # Uses the same i->y, j->x convention as cell_to_map.
        
        
        ii,jj = np.meshgrid(np.arange(rows),np.arange(cols), indexing = 'ij')
        
        x_map = self.grid_origin[0] + jj*self.grid_resolution + self.grid_resolution / 2
        y_map = self.grid_origin[1] + ii*self.grid_resolution + self.grid_resolution / 2
        
        self.cell_to_map_coords = np.stack([x_map,y_map],axis= -1)
        
        self.probability_grid += self.gaussian_bump(map_xy=self.prior_centroid, alpha= 1, sigma= self.arena_size[0] / 5)
        
        return None
        
        
    # ----------------------------------------------------------------------
    # Coordinate conversion
    # ----------------------------------------------------------------------
    def map_to_cell(self, x: float, y: float) -> tuple:
        """Convert (x, y) in map frame to (i, j) grid indices."""
        j = int((x - self.grid_origin[0])/self.grid_resolution)
        i = int((y-self.grid_origin[1])/self.grid_resolution)
        return i,j

    def cell_to_map(self, i: int, j: int) -> tuple:
        """Convert (i, j) grid indices to (x, y) in map frame (cell center)."""
        x_map = self.grid_origin[0] + j*self.grid_resolution + self.grid_resolution/2
        y_map = self.grid_origin[1] + i*self.grid_resolution + self.grid_resolution/2

        return x_map,y_map

    # ----------------------------------------------------------------------
    # FLS geometry resolution (lazy, needs tf2 buffer populated)
    # ----------------------------------------------------------------------
    def set_geometry(self) -> bool:
        """
        Resolve the FLS viewing geometry once the map->ned TF is available.
        seafloor_depth is the NED depth of the map origin (a seafloor point at
        map z=0), so for the origin point it is just the ned<-map translation z.
        Returns True if geometry is ready, False if the TF isn't available yet.
        """
        if self.seafloor_depth is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform('ned', 'map', rclpy.time.Time())
        except tf2_ros.TransformException:
            return False  # TF not ready yet

        # cache the full ned<-map transform so cell_to_search_pose can convert
        # cell centres to NED without a per-call tf2 lookup (the TF is static).
        q = tf.transform.rotation
        t = tf.transform.translation
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = [t.x, t.y, t.z]
        self.T_ned_map = T

        self.seafloor_depth = t.z   # NED depth of map origin (seafloor at map z=0)
        self.clearance = self.seafloor_depth - self.search_depth
        self.x_adjustment = self.clearance / np.tan(self.mount_angle)
        return True
        

    def camera_info_callback(self, msg: CameraInfo):
        # only call once as intrinsics do not change, note self.camera_info is set to None initially
        if self.camera_info is None:
            self.camera_info = msg # we use for checking
            self.fx = msg.k[0]   # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
                
            
            
    # ----------------------------------------------------------------------
    # Subscription callback: continuous Bayesian update from FLS
    # ----------------------------------------------------------------------
    def depth_callback(self, msg: Image):
        """
        On each FLS depth image:
          1. Determine which cells the sensor footprint covered (back projection)
          2. Mark visited_grid[covered] = True
          3. Apply Bayesian update: probability_grid[covered] = 0
             (deterministic sensor: covered cells with no target → P_target = 0)
        """
        if self.camera_info is None:
            return # wait for intrinsics
        
        # need optical relative to map transform
        
        try:
            tf = self.tf_buffer.lookup_transform('map', 'sonar_optical', rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        
        # parse depth image (32FC1, single channel float32)
        depth = np.frombuffer(msg.data,dtype=np.float32).reshape(msg.height,msg.width)
        
        # valid returns only (0.0 == no hit, per sensor logic)
        vs,us = np.where(depth > 0.0)
        if vs.size == 0:
            return
        ds = depth[vs,us] # depth reading 1D array
        
        # back-project to sonar optical frame (pinhole model with intrinsics), element wise multiplication
        X = (us - self.cx) * ds/ self.fx
        Y = (vs - self.cy) * ds/ self.fy
        Z = ds
        
        #stack into N x 3 matrix
        pts_optical = np.stack([X,Y,Z], axis = -1)  #(N,3)
        
        # express the optical to map transform as 4x4 SE(3)
        q = tf.transform.rotation
        t = tf.transform.translation
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])  # 4x4
        T[:3, 3] = [t.x, t.y, t.z]
        
        # need (N,3) to become (N,4) homogeneous coordinates by appending a column of ones
        # so a point h is x,y,z,1 along the row
        pts_h = np.hstack([pts_optical, np.ones((pts_optical.shape[0], 1))])  # (N,4)

        # left-multiply, column-vector convention: re-express each point in map frame.
        # T @ pts_h.T : (4,4) @ (4,N) becomes (4,N), columns are transformed points; transpose back to (N,4)
        pts_map = (T @ pts_h.T).T[:, :3]   # drop homogeneous coord , becomes (N,3) map points

        # map (x,y) -> grid (i,j); x->j (col), y->i (row)
        js = ((pts_map[:, 0] - self.grid_origin[0]) / self.grid_resolution).astype(int)
        iss = ((pts_map[:, 1] - self.grid_origin[1]) / self.grid_resolution).astype(int)

        # keep only points landing inside the grid (terrain extends beyond the arena)
        rows, cols = self.grid_shape
        in_bounds = (iss >= 0) & (iss < rows) & (js >= 0) & (js < cols)
        iss, js = iss[in_bounds], js[in_bounds]

        # mark swept + zero belief (deterministic sensor: covered with no target -> 0)
        self.visited_grid[iss, js] = True
        self.probability_grid[iss, js] = 0.0


    # ----------------------------------------------------------------------
    # Service callback: cluster bump on new completions + return next pose
    # ----------------------------------------------------------------------
    def get_next_pose_callback(self, request, response):
        """
        1. Cluster-bump for each NEW completed target (delta via bumped_targets_set)
        2. utility = score / cost 
        3. argmax over the grid -> (i, j)
        4. cell_to_search_pose(i, j) -> response.next_pose (NED, FLS-offset)
        """
        # --- 1. cluster bumps for newly completed targets ---
        for target in request.visited_targets:
            target_id = target.header.frame_id      # geometry ID rides in frame_id
            if target_id in self.bumped_targets_set:
                continue                            # already bumped -> skip (delta)
            # position is map coords by convention (frame_id is the ID, not a tf frame)
            x = target.pose.position.x
            y = target.pose.position.y
            # bump sigma ~ tether length / 2 (50 m tether -> 25 m); fixed
            self.probability_grid += self.gaussian_bump((x, y), alpha=1.0, sigma=25.0)
            self.bumped_targets_set.add(target_id)

        # keep swept cells at zero after any bump (bumps are purely additive)
        self.probability_grid *= ~self.visited_grid

        # --- 2. utility = score / cost (approximate-Dubins cost) ---
        cost = self.compute_cost_grid()
        utility = self.probability_grid / (cost + 1e-6)   # epsilon avoids /0 at vehicle

        # --- 3. argmax over the grid ---
        i, j = np.unravel_index(np.argmax(utility), utility.shape)

        # --- 4. build the response ---
        response.next_pose = self.cell_to_search_pose(int(i), int(j))
        response.cell_utility = float(utility[i, j])
        response.search_exhausted = bool(np.all(self.visited_grid))
        return response

    # ----------------------------------------------------------------------
    # Cell -> search pose (NED, north-facing, offset for forward-tilted FLS)
    # ----------------------------------------------------------------------
    def cell_to_search_pose(self, i: int, j: int) -> PoseStamped:
        """
        Build the vehicle pose that places the FLS centre beam on cell (i, j).
        Cell centre (map, seafloor z=0) -> NED via the cached T_ned_map, then the
        vehicle sits x_adjustment behind (north of) the cell at search_depth,
        facing north.
        """
        x_map, y_map = self.cell_to_map(i, j)
        p_map = np.array([x_map, y_map, 0.0, 1.0])   # seafloor point, homogeneous
        p_ned = self.T_ned_map @ p_map               # (4,) cell centre in NED
        north, east = p_ned[0], p_ned[1]

        pose = PoseStamped()
        pose.header.frame_id = 'ned'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(north - self.x_adjustment)  # behind cell for FLS tilt
        pose.pose.position.y = float(east)
        pose.pose.position.z = float(self.search_depth)
        pose.pose.orientation.w = 1.0                            # north-facing in NED
        return pose

    # ----------------------------------------------------------------------
    # Approximate-Dubins cost grid: Euclidean distance + yaw penalty
    # ----------------------------------------------------------------------
    def compute_cost_grid(self) -> np.ndarray:
        """
        Per-cell travel cost ~ distance + turning_radius * |yaw_diff|.
        Distance is Euclidean in the map frame (cells cached there, 2D since all
        cells share the same z). Search cells are north-facing (yaw_target = 0 in
        NED), so the yaw term reduces to the vehicle's NED yaw magnitude --
        constant across cells, but it shifts the score/distance trade-off when the
        vehicle is misaligned. Both terms are scalars by the time they combine, so
        the map/NED frame mix is harmless (rigid transforms preserve distance/angle).
        """
        try:
            # vehicle position in map (distance term)
            veh_map = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            vx = veh_map.transform.translation.x
            vy = veh_map.transform.translation.y

            # vehicle yaw in NED (target heading is north = 0)
            veh_ned = self.tf_buffer.lookup_transform('ned', 'base_link', rclpy.time.Time())
            q = veh_ned.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        except tf2_ros.TransformException as ex:
            # vehicle pose unavailable -> fall back to uniform cost so utility
            # reduces to pure score (search still progresses)
            self.get_logger().warn(f"cost: vehicle TF unavailable ({ex}); using uniform cost")
            return np.ones(self.grid_shape)

        # Euclidean distance over the whole grid (vectorized, map frame)
        diff = self.cell_to_map_coords - np.array([vx, vy])   # (n,n,2) - (2,)
        distance = np.linalg.norm(diff, axis=-1)              # (n,n)

        # yaw penalty: turning_radius * |yaw| (scalar, broadcasts over the grid)
        yaw_penalty = self.turning_radius * abs(yaw)

        return distance + yaw_penalty

    # ----------------------------------------------------------------------
    # gaussian helper
    # ----------------------------------------------------------------------
    def gaussian_bump(self, map_xy: tuple[float,float], alpha: float, sigma: float) -> np.ndarray:
        """Generate a Gaussian centered at world_xy on the grid for cluster discovery."""

        #subtract the coordinates from the centroid
        diff = self.cell_to_map_coords - np.array(map_xy) # have (n,n,2) - (2,) broadcast
        dist_squared = np.sum(diff**2, axis = -1) #sum across the last axis
        gaussian = alpha * np.exp(-dist_squared/(2*sigma**2))
        return gaussian

    # ----------------------------------------------------------------------
    # Visualization: publish the belief as an OccupancyGrid for RViz
    # ----------------------------------------------------------------------
    def publish_belief(self):
        """
        Publish probability_grid as a nav_msgs/OccupancyGrid (map frame).
        Values are normalized to [0, 100] (int8). Visualization only -- it does
        not feed back into the algorithm.
        """
        rows, cols = self.grid_shape

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'                  # grid is map-aligned; RViz overlays it
        msg.info.resolution = self.grid_resolution
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = float(self.grid_origin[0])
        msg.info.origin.position.y = float(self.grid_origin[1])
        msg.info.origin.position.z = 1.0   # float above the coplanar terrain so it's visible
        msg.info.origin.orientation.w = 1.0

        # normalize to [0, 100]; data is row-major (i*cols + j), matching the grid
        g = self.probability_grid
        max_val = g.max()
        if max_val > 0:
            norm = (g / max_val * 100.0).astype(np.int8)
        else:
            norm = np.zeros_like(g, dtype=np.int8)
        msg.data = norm.flatten().tolist()

        self.belief_pub.publish(msg)


def main(args=None):
    import time

    rclpy.init(args=args)
    node = BayesianSearchServer()

    # Resolve the static FLS geometry before serving any request. spin_once runs
    # the TransformListener callback (populating the buffer from the latched
    # /tf_static), so this is non-blocking and resolves within a tick or two.
    deadline = time.time() + 10.0
    while rclpy.ok() and not node.set_geometry():
        if time.time() > deadline:
            node.get_logger().error(
                "map->ned TF never arrived; check launch / static TF publishers")
            node.destroy_node()
            rclpy.shutdown()
            return
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        f"FLS geometry resolved: seafloor_depth={node.seafloor_depth:.2f}, "
        f"clearance={node.clearance:.2f}, x_adjustment={node.x_adjustment:.2f}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()