from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of the gripper'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate of the gripper'
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='1.0',
        description='Timeout (connection) of the gripper'
    )
    action_timeout_arg = DeclareLaunchArgument(
        'action_timeout',
        default_value='20',
        description='Action timeout (movement) of the gripper'
    )
    slave_address_arg = DeclareLaunchArgument(
        'slave_address',
        default_value='9',
        description='Slave address of the gripper'
    )
    fake_hardware_arg = DeclareLaunchArgument(
        'fake_hardware',
        default_value='False',
        description='Fake hardware'
    )

    robotiq_2f_gripper_node = Node(
        package='robotiq_2f_gripper_hardware',
        executable='robotiq_2f_gripper_node',
        name='robotiq_2f_gripper_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout': LaunchConfiguration('timeout'),
            'action_timeout': LaunchConfiguration('action_timeout'),
            'slave_address': LaunchConfiguration('slave_address'),
            'fake_hardware': LaunchConfiguration('fake_hardware')
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        timeout_arg,
        action_timeout_arg,
        slave_address_arg,
        fake_hardware_arg,
        robotiq_2f_gripper_node
    ])