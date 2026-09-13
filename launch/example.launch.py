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

                # Command authority. Start conservative and raise once the
                # vehicle behaves; these clamps are the only limit on the
                # firmware's full-range input.
                'limits.brake_min': 0.0,
                'limits.brake_max': 0.30,
                'limits.throttle_min': 0.0,
                'limits.throttle_max': 0.15,
                'limits.steering_torque_min': -0.25,
                'limits.steering_torque_max': 0.25,

                # Steering wheel angle out of the OBD stream.
                # Defaults are the Kia Soul values from OSCC's vehicles.h.
                # Set can_id / scale / byte order for your own vehicle.
                'steering_feedback.enabled': True,
                'steering_feedback.can_id': 0x2B0,
                'steering_feedback.start_bit': 0,
                'steering_feedback.bit_length': 16,
                'steering_feedback.little_endian': True,
                'steering_feedback.is_signed': True,
                'steering_feedback.scale': 0.1,
                'steering_feedback.offset': 0.0,
                'steering_feedback.min_angle': -720.0,
                'steering_feedback.max_angle': 720.0,

                # Brake pedal. bit_length=1 reads a switch; widen it and set
                # press_threshold to read a pressure signal instead.
                'brake_pedal_feedback.enabled': True,
                'brake_pedal_feedback.can_id': 0x220,
                'brake_pedal_feedback.start_bit': 0,
                'brake_pedal_feedback.bit_length': 1,
                'brake_pedal_feedback.little_endian': True,
                'brake_pedal_feedback.is_signed': False,
                'brake_pedal_feedback.scale': 1.0,
                'brake_pedal_feedback.offset': 0.0,
                'brake_pedal_feedback.press_threshold': 0.5,
                'brake_pedal_feedback.active_high': True,

                'feedback_timeout': 1.0,
            }],
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='roscco',
            executable='roscco_teleop',
            name='roscco_teleop',
            parameters=[{
                'brake_axis': 4,
                'throttle_axis': 5,
                'steering_axis': 2,
                'start_button': 0,
                'back_button': 1,
                'publish_rate_hz': 50.0,
                'joy_timeout': 0.3,
                'steering_smoothing_factor': 0.1,
                'max_brake': 1.0,
                'max_throttle': 1.0,
                'max_steering_torque': 1.0,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
