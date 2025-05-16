from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    config_path = '/home/miskota/dvrk2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json'

    arm_node = Node(
        package='ros2_course',
        executable='arm_node',
        name='arm_node',
        #output='screen'
    )

    xbox_controller = Node(
        package='ros2_course',
        executable='xbox_controller',
        name='xbox_controller',
        #output='screen'
    )

    interactive_marker = Node(
        package='ros2_course',
        executable='interactive_marker',
        name='interactive_marker',
        #output='screen'
    )

    dvrk_console_json = Node(
        package='dvrk_robot',
        executable='dvrk_console_json',
        name='dvrk_console_json',
        arguments=['-j', config_path],
        #output='screen'
    )

    
    state_publisher = ExecuteProcess(
        cmd=['ros2', 'launch', 'dvrk_model', 'dvrk_state_publisher.launch.py', 'arm:=PSM1'],
        #output='screen'
    )

    
    rviz = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rviz2', 'rviz2',
            '-d', '/home/miskota/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz'
        ],
        #output='screen'
    )

    
    static_tf_world_to_camera = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--frame-id', 'world',
            '--child-frame-id', 'camera',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1'
        ],
        output='screen'
    )

    
    static_tf_psm_to_camera = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--frame-id', 'PSM1_psm_base_link',
            '--child-frame-id', 'camera',
            '--x', '0.18', '--y', '0.03', '--z', '0.01',
            '--roll', '2.70526034',
            '--pitch', '-0.78539816',
            '--yaw', '-2.53072742'
        ],
        output='screen'
    )

    return LaunchDescription([
        arm_node,
        xbox_controller,
        interactive_marker,
        dvrk_console_json,
        state_publisher,
        rviz,
        static_tf_world_to_camera,
        static_tf_psm_to_camera
    ])
