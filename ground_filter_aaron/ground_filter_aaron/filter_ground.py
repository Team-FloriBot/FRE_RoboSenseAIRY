#!/usr/bin/env python3
"""
ROS 2 ground filter pipeline:
- Subscribes to raw PointCloud2
- Removes ground with RANSAC + morphology (erosion then dilation)
- Publishes filtered cloud and LaserScan projection
- Can optionally play an MCAP bag for offline testing
"""

import subprocess
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def _filter_by_azimuth(
    points: np.ndarray,
    angle_range_deg: float = 45.0,
) -> np.ndarray:
    """
    Filter points by horizontal (azimuth) angle from sensor.
    Keep only points within ±angle_range_deg from the forward direction (x-axis).
    
    Args:
        points: Array of shape (N, 3) with xyz coordinates
        angle_range_deg: Keep points within ±this angle from forward (x-axis)
        
    Returns:
        Boolean mask of shape (N,) where True means keep the point
    """
    if points.shape[0] == 0:
        return np.array([], dtype=bool)
    
    # Calculate azimuth angle (horizontal angle in xy-plane)
    angles_rad = np.arctan2(points[:, 1], points[:, 0])
    
    # Convert to degrees and normalize to [-180, 180]
    angles_deg = np.degrees(angles_rad)
    
    # Keep points within ±angle_range_deg from forward direction (0°)
    angle_range_rad = np.radians(angle_range_deg)
    mask = np.abs(angles_rad) <= angle_range_rad
    
    return mask


def _fit_plane_ransac(
    points: np.ndarray,
    rng: np.random.Generator,
    iterations: int = 90,
    distance_threshold: float = 0.16,
) -> tuple[np.ndarray, float, np.ndarray] | None:
    """Fit dominant plane with RANSAC. Returns (normal, d, inlier_mask)."""
    if points.shape[0] < 3:
        return None

    best_count = -1
    best_normal = None
    best_d = None
    best_mask = None

    for _ in range(iterations):
        ids = rng.choice(points.shape[0], size=3, replace=False)
        p0, p1, p2 = points[ids]

        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal = normal / norm

        d = -float(np.dot(normal, p0))
        distances = np.abs(points @ normal + d)
        inlier_mask = distances < distance_threshold
        count = int(inlier_mask.sum())

        if count > best_count:
            best_count = count
            best_normal = normal
            best_d = d
            best_mask = inlier_mask

    if best_normal is None:
        return None

    return best_normal, best_d, best_mask


def _make_plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Create two orthonormal vectors spanning the plane."""
    ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(normal, ref))) > 0.9:
        ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    u = np.cross(normal, ref)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    v /= np.linalg.norm(v)
    return u, v

"""
Build ground mask by thresholding distance to plane and cleaning in 2D grid.
"""
def _ground_mask_by_grid(
    points: np.ndarray,
    normal: np.ndarray,
    d: float,
    distance_threshold: float = 0.20,
    cell_size: float = 0.25
) -> np.ndarray:

    ##################################################################
    # Step 1: calculate distance to plane and initial candidate mask
    ##################################################################
    distances = np.abs(points @ normal + d)
    candidate = distances < distance_threshold
    if candidate.sum() < 100:
        return candidate

    ##################################################################
    # Step 2: create 2D grid in plane coordinates and count points per cell
    ##################################################################
    # calculate orthonormal vectors on the plane
    u, v = _make_plane_basis(normal)
    u_coord = points @ u
    v_coord = points @ v

    # determine grid bounds and size
    min_u = float(u_coord.min())
    min_v = float(v_coord.min())
    max_u = float(u_coord.max())
    max_v = float(v_coord.max())

    # number of cells in each direction (with safety limits)
    nu = int(np.floor((max_u - min_u) / cell_size)) + 1
    nv = int(np.floor((max_v - min_v) / cell_size)) + 1
    if nu <= 0 or nv <= 0 or nu * nv > 4_000_000:
        return candidate

    # assign points to grid cells
    iu = np.floor((u_coord - min_u) / cell_size).astype(np.int32)
    iv = np.floor((v_coord - min_v) / cell_size).astype(np.int32)
    iu = np.clip(iu, 0, nu - 1)
    iv = np.clip(iv, 0, nv - 1)

    # Count total points and candidate points in each cell
    all_count = np.zeros((nu, nv), dtype=np.uint16)
    can_count = np.zeros((nu, nv), dtype=np.uint16)
    np.add.at(all_count, (iu, iv), 1)
    np.add.at(can_count, (iu[candidate], iv[candidate]), 1)

    ##################################################################
    # Step 3: determine which cells are ground-like based on candidate ratio
    ##################################################################
    # A cell is ground-like if most of its points are close to the plane.
    ratio = np.zeros((nu, nv), dtype=np.float32)
    nonzero = all_count > 0
    ratio[nonzero] = can_count[nonzero] / all_count[nonzero]
    cell_ground = (can_count >= 2) & (ratio > 0.55)

    # Mark point as ground only if it is close to plane and inside cleaned ground region.
    return candidate & cell_ground[iu, iv]

"""
Filter ground by checking slope between concentric distance rings.
Implements radial slope check: |tan(β_max)| > (Δa_ist / Δg_ist)
where Δa = vertical height change, Δg = horizontal distance change.

Points organized by distance from sensor (xy-plane projection).
For each azimuth sector, slope between consecutive rings is checked.
Steep transitions (> max_slope_deg) mark obstacles.

Args:
    points: (N, 3) point cloud
    max_slope_deg: max allowed slope between rings (degrees)
    ring_width: distance in meters per ring
    
Returns:
    Boolean mask where True = ground, False = obstacle
"""
def _ground_mask_by_slope(
    points: np.ndarray,
    max_slope_deg: float = 15.0,
    ring_width: float = 0.5,
) -> np.ndarray:

    if points.shape[0] == 0:
        return np.array([], dtype=bool)

    # Distance in xy-plane
    xy_dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)

    # Azimuth angle
    azimuth = np.arctan2(points[:, 1], points[:, 0])

    # Height
    height = points[:, 2]

    # Initially assume all points are ground
    ground_mask = np.ones(points.shape[0], dtype=bool)

    if xy_dist.max() < ring_width:
        return ground_mask

    # -----------------------------
    # Create radial rings
    # -----------------------------
    n_rings = int(np.ceil(xy_dist.max() / ring_width)) + 1

    ring_indices = np.floor(xy_dist / ring_width).astype(np.int32)
    ring_indices = np.clip(ring_indices, 0, n_rings - 1)

    # -----------------------------
    # Create azimuth sectors
    # -----------------------------
    azimuth_deg = np.degrees(azimuth)

    n_sectors = 45
    sector_size = 360.0 / n_sectors

    sector_indices = np.floor(
        (azimuth_deg + 180.0) / sector_size
    ).astype(np.int32)

    sector_indices = np.clip(
        sector_indices,
        0,
        n_sectors - 1
    )

    # Max allowed slope
    max_slope_tan = abs(
        np.tan(np.radians(max_slope_deg))
    )

    # =========================================================
    # Process each azimuth sector independently
    # =========================================================
    for sector in range(n_sectors):

        sector_mask = sector_indices == sector

        if sector_mask.sum() < 2:
            continue

        sector_idx = np.where(sector_mask)[0]

        sect_rings = ring_indices[sector_idx]
        sect_heights = height[sector_idx]
        sect_distances = xy_dist[sector_idx]

        # -----------------------------------------------------
        # Find first valid ring as initial ground reference
        # -----------------------------------------------------
        last_ground_height = None
        last_ground_distance = None

        for ring in range(n_rings):
            #if ring == 0:
                #print("######################0######################")

            curr_ring_mask = sect_rings == ring

            if curr_ring_mask.sum() == 0:
                #print(f"Sector {sector}, ring {ring}: no points, skipping")
                continue

            # Robust statistics
            h_curr = np.percentile(
                sect_heights[curr_ring_mask],
                10
            )

            d_curr = np.percentile(
                sect_distances[curr_ring_mask],
                10
            )

            curr_point_idx = sector_idx[curr_ring_mask]

            # ---------------------------------------------
            # First valid ring initializes ground model
            # ---------------------------------------------
            if last_ground_height is None:
                if curr_ring_mask.sum() < 40:
                    continue

                # robustere Initialisierung
                candidate_h = np.percentile(sect_heights[curr_ring_mask], 10)
                last_ground_height = candidate_h
                last_ground_distance = d_curr
                continue

            # ---------------------------------------------
            # Compare against LAST VALID GROUND
            # ---------------------------------------------
            delta_h = h_curr - last_ground_height
            delta_d = d_curr - last_ground_distance

            if abs(delta_d) < 1e-6:
                #print("delta_d too small, skipping slope check")
                continue

            slope_ratio = abs(delta_h / delta_d)

            # ---------------------------------------------
            # Too steep -> obstacle
            # ---------------------------------------------
            if slope_ratio > max_slope_tan:

                ground_mask[curr_point_idx] = False

            else:
                # -----------------------------------------
                # Valid ground -> update reference
                # -----------------------------------------
                last_ground_height = h_curr
                last_ground_distance = d_curr

    return ground_mask


class GroundFilterNode(Node):
    """Publishes a nonground PointCloud2 and a LaserScan projection."""

    def __init__(self):
        super().__init__('filter_ground')

        self._declare_parameters()

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)

        self.azimuth_fov = float(self.get_parameter("azimuth_fov_deg").value)
        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.distance_threshold = float(self.get_parameter("distance_threshold").value)
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.use_ransac_filter = bool(self.get_parameter("use_ransac_filter").value)

        self.max_slope_deg = float(self.get_parameter("max_slope_deg").value)
        self.slope_ring_width = float(self.get_parameter("slope_ring_width").value)

        scan_angle_increment_deg = float(
            self.get_parameter("scan_angle_increment_deg").value
        )
        self.scan_angle_increment = np.radians(scan_angle_increment_deg)
        self.scan_range_min = float(self.get_parameter("scan_range_min").value)
        self.scan_range_max = float(self.get_parameter("scan_range_max").value)

        self.bag_play_enabled = bool(self.get_parameter("bag_play_enabled").value)
        self.bag_file = str(self.get_parameter("bag_file").value)
        self.bag_loop = bool(self.get_parameter("bag_loop").value)

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.scan_publisher = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._on_cloud,
            10,
        )

        self.bag_process = None
        self._rng = np.random.default_rng()
        self._frame_counter = 0

        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Filtered topic: {self.output_topic}')
        self.get_logger().info(f'Filtered scan topic: {self.scan_topic}')

        if self.bag_play_enabled:
            self._start_bag_playback()

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("output_topic", "/rslidar_points_filtered")
        self.declare_parameter("scan_topic", "/rslidar_points_filtered_scan")

        self.declare_parameter("azimuth_fov_deg", 100.0)
        self.declare_parameter("ransac_iterations", 90)
        self.declare_parameter("distance_threshold", 0.05)
        self.declare_parameter("cell_size", 0.25)
        self.declare_parameter("use_ransac_filter", True)

        self.declare_parameter("max_slope_deg", 35.0)
        self.declare_parameter("slope_ring_width", 0.10)

        self.declare_parameter("scan_angle_increment_deg", 0.5)
        self.declare_parameter("scan_range_min", 0.05)
        self.declare_parameter("scan_range_max", 100.0)

        self.declare_parameter("bag_play_enabled", False)
        self.declare_parameter("bag_file", "")
        self.declare_parameter("bag_loop", True)

    def _start_bag_playback(self) -> None:
        bag_file = self.bag_file
        if not bag_file:
            mcap_files = list(Path('/app').glob('*.mcap'))
            if not mcap_files:
                mcap_files = list(Path('.').glob('*.mcap'))

            if not mcap_files:
                raise FileNotFoundError(
                    "bag_play_enabled is true, but no .mcap file was found."
                )

            bag_file = str(mcap_files[0])

        command = ['ros2', 'bag', 'play']
        if self.bag_loop:
            command.append('-l')
        command.append(bag_file)

        try:
            self.get_logger().info(f'Starting ros2 bag play: {bag_file}')
            self.bag_process = subprocess.Popen(command)
        except Exception as exc:
            raise RuntimeError(f'Could not start ros2 bag play: {exc}') from exc

    def _publish_filtered_outputs(self, header, points: np.ndarray) -> None:
        filtered_msg = point_cloud2.create_cloud_xyz32(
            header,
            points.tolist(),
        )
        self.publisher.publish(filtered_msg)
        self.scan_publisher.publish(self._points_to_laserscan(header, points))

    def _points_to_laserscan(self, header, points: np.ndarray) -> LaserScan:
        scan = LaserScan()
        scan.header = header
        scan.angle_min = float(np.radians(-self.azimuth_fov))
        scan.angle_max = float(np.radians(self.azimuth_fov))
        scan.angle_increment = float(self.scan_angle_increment)
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = float(self.scan_range_min)
        scan.range_max = float(self.scan_range_max)

        bin_count = int(np.floor((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        ranges = np.full(bin_count, np.inf, dtype=np.float32)

        if points.shape[0] > 0:
            xy_ranges = np.hypot(points[:, 0], points[:, 1])
            angles = np.arctan2(points[:, 1], points[:, 0])
            valid = (
                (xy_ranges >= self.scan_range_min)
                & (xy_ranges <= self.scan_range_max)
                & (angles >= scan.angle_min)
                & (angles <= scan.angle_max)
            )

            if valid.any():
                indices = np.floor((angles[valid] - scan.angle_min) / scan.angle_increment).astype(np.int32)
                indices = np.clip(indices, 0, bin_count - 1)
                np.minimum.at(ranges, indices, xy_ranges[valid])

        scan.ranges = ranges.tolist()
        return scan

    def _on_cloud(self, msg: PointCloud2) -> None:
        # Process every incoming frame sequentially
        self._frame_counter += 1
        self.get_logger().debug(f'Processing frame #{self._frame_counter}...')

        points_raw = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

        if isinstance(points_raw, np.ndarray) and points_raw.dtype.names:
            points = np.column_stack(
                (points_raw['x'], points_raw['y'], points_raw['z'])
            ).astype(np.float32, copy=False)
        else:
            points = np.asarray(list(points_raw), dtype=np.float32)

        if points.shape[0] < 50:
            # Too few points: publish what we have (possibly empty) and continue
            self._publish_filtered_outputs(msg.header, points)
            return

        # Filter by azimuth FIRST: keep only ±120° forward direction
        azimuth_mask = _filter_by_azimuth(points, angle_range_deg=self.azimuth_fov)
        points_fov = points[azimuth_mask]

        if points_fov.shape[0] < 50:
            self._publish_filtered_outputs(msg.header, points_fov)
            return

        # Choose filtering method
        if self.use_ransac_filter:
            # --- Method 1: RANSAC + Morphology ---
            model = _fit_plane_ransac(
                points_fov,
                rng=self._rng,
                iterations=self.ransac_iterations,
                distance_threshold=self.distance_threshold,
            )

            if model is None:
                # Keep only azimuth-filtered points if plane fitting failed.
                self._publish_filtered_outputs(msg.header, points_fov)
                return

            normal, d, _ = model
            ground_mask = _ground_mask_by_grid(
                points_fov,
                normal,
                d,
                distance_threshold=self.distance_threshold,
                cell_size=self.cell_size
            )
            method_name = "RANSAC"
        else:
            # --- Method 2: Slope-based radial filter ---
            ground_mask = _ground_mask_by_slope(
                points_fov,
                max_slope_deg=self.max_slope_deg,
                ring_width=self.slope_ring_width,
            )
            method_name = "Slope-based"

        nonground = points_fov[~ground_mask]

        # Publish filtered cloud and its 2D LaserScan projection for this frame
        self._publish_filtered_outputs(msg.header, nonground)

        if self._frame_counter % 20 == 0:
            removed_azimuth = int((~azimuth_mask).sum())
            removed_ground = int(ground_mask.sum())
            total = int(points.shape[0])
            final = int(nonground.shape[0])
            self.get_logger().info(
                f'Frame {self._frame_counter}: {total} → {final} points '
                f'({method_name}, azimuth: -{removed_azimuth}, ground: -{removed_ground}).'
            )

    def destroy_node(self) -> bool:
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.bag_process.terminate()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

