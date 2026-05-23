from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ground_segmentation')
    rviz_config = os.path.join(pkg_share, 'config', 'point_visual.rviz')

    bag_path = os.path.expanduser('~/rosbag2_2026_05_22-17_49_45')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--loop'],
            output='screen'
        ),

        Node(
            package='ground_segmentation',
            executable='ground_segmentation_node',
            name='ground_segmentation_node',
            output='screen',
            parameters=[{
                'input_topic': '/rslidar_points',
                'aligned_topic': '/aligned_points',
                'ground_topic': '/ground_points',
                'nonground_topic': '/nonground_points',
                'crop_topic': '/crop_points',
                'obstacle_topic': '/obstacle_points',

                'parent_frame': 'base_link',
                'sensor_frame': 'rslidar',

                'mount_x': 0.0,
                'mount_y': 0.0,
                'mount_z': 0.50,

                'initial_pitch_deg': -20.0,
                'initial_roll_deg': 0.0,

                'leveling_pitch_sign': 1.0,
                'leveling_roll_sign': 1.0,
                'leveling_pitch_gain': 1.0,
                'leveling_roll_gain': 1.0,

                'use_ring_filter': False,
                'ring_min': 30,
                'ring_max': 120,

                'use_local_ground_leveling': True,
                'leveling_update_every_n_frames': 1,
                'leveling_stride': 4,
                'leveling_alpha': 0.25,

                'iterative_leveling_iterations': 4,
                'iterative_leveling_step_gain': 1.0,
                'iterative_leveling_convergence_deg': 0.2,

                'max_pitch_correction_deg': 35.0,
                'max_roll_correction_deg': 35.0,

                'leveling_roi_x_min': 0.2,
                'leveling_roi_x_max': 2.5,
                'leveling_roi_y_min': -1.2,
                'leveling_roi_y_max': 1.2,
                'leveling_roi_z_min': -1.2,
                'leveling_roi_z_max': 2.0,
                'leveling_grid_resolution': 0.10,
                'leveling_min_cells': 6,
                'leveling_neighbor_radius': 1,

                'publish_aligned_cloud': True,

                'roi_x_min': 0.0,
                'roi_x_max': 6.0,
                'roi_y_min': -2.0,
                'roi_y_max': 2.0,
                'roi_z_min': -1.0,
                'roi_z_max': 2.0,

                'grid_resolution': 0.08,
                'min_points_per_cell': 2,
                'local_plane_neighbor_radius': 1,
                'local_plane_min_cells': 5,
                'base_ground_threshold': 0.05,
                'distance_threshold_coeff': 0.01,
                'negative_outlier_threshold': 0.10,
                'obstacle_height_span_threshold': 0.12,
                'ground_band_height': 0.04,

                'enable_crop_obstacle_split': True,
                'cluster_tolerance': 0.24,
                'cluster_min_size': 2,
                'cluster_max_size': 20000,
                
                'crop_min_height': 0.1,
                'crop_max_height': 0.50,
                'crop_max_ground_offset': 0.2,


            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_point_visual',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
