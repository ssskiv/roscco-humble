from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_channel = LaunchConfiguration('can_channel')
    joy_dev = LaunchConfiguration('joy_dev')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_channel', default_value='0',
            description='SocketCAN channel index: 0 means can0.'),
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Joystick device node.'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_name': '',
                'dev': joy_dev,
                # Publish even when nothing changes, so the teleop watchdog
                # sees a steady heartbeat rather than silence.
                'autorepeat_rate': 50.0,
                'deadzone': 0.05,
            }],
            output='screen',
        ),

        Node(
            package='roscco',
            executable='roscco_node',
            name='roscco_node',
            parameters=[{
                'can_channel': can_channel,
                'drain_period_ms': 5,
            }],
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='roscco',
            executable='roscco_teleop',
            name='roscco_teleop',
            parameters=[{
                'brake_axis': 2,
                'throttle_axis': 5,
                'steering_axis': 0,
                'start_button': 7,
                'back_button': 6,
                'publish_rate_hz': 50.0,
                'joy_timeout': 0.3,
                'steering_smoothing_factor': 0.1,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
