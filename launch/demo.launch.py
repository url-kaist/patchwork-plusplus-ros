import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('patchworkpp'),
    'config',
    'params_ros2.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value="/kitti/point_cloud"),
        
        Node(
            package='patchworkpp',
            executable='demo',
            name='ground_segmentation',
            output='screen',
            parameters=[
                {'cloud_topic': '/points'}, # Input pointcloud
                {'frame_id': 'laser_data_frame'},
                {'sensor_height': 1.88},
                {'num_iter': 3},             # Number of iterations for ground plane estimation using PCA.
                {'num_lpr': 20},             # Maximum number of points to be selected as lowest points representative.
                {'num_min_pts': 0},          # Minimum number of points to be estimated as ground plane in each patch.
                {'th_seeds': 0.3},           # threshold for lowest point representatives using in initial seeds selection of ground points.
                {'th_dist': 0.125},          # threshold for thickenss of ground.
                {'th_seeds_v': 0.25},        # threshold for lowest point representatives using in initial seeds selection of vertical structural points.
                {'th_dist_v': 0.9},          # threshold for thickenss of vertical structure.
                {'max_r': 80.0},             # max_range of ground estimation area
                {'min_r': 1.0},              # min_range of ground estimation area
                {'uprightness_thr': 0.101},  # threshold of uprightness using in Ground Likelihood Estimation(GLE). Please refer paper for more information about GLE.
                {'verbose': False},          # display verbose info
                {'display_time': False},     # display running_time and pointcloud sizes
            ],
            arguments=[LaunchConfiguration('cloud_topic')],
        ),
    ])
 


