from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


parameters_file_path = Path(
    get_package_share_directory('vrpn_client_ros'),
    'config',
    'sample.params.yaml'
)


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vrpn_client_ros',
            executable='vrpn_client_node',
            output='screen',
            emulate_tty=True,
            parameters=[parameters_file_path],
        ),
    ])