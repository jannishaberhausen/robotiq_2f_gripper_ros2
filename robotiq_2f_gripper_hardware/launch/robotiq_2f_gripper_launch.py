import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable


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
        description='Whether to use fake hardware (if real hardware is not available)'
    )
    rviz2_arg = DeclareLaunchArgument(
        'rviz2',
        default_value='False',
        description='Wether to launch rviz2 for simultaneous visualization'
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

    urdf_file = os.path.join(get_package_share_directory('robotiq_2f_gripper_description'), 'urdf', 'example_use_robotiq_2f_140.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory('robotiq_2f_gripper_description'), 'launch', 'robotiq_2f_gripper.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }],
        condition=IfCondition(LaunchConfiguration('rviz2'))
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['/robotiq_2f_gripper/joint_states']
        }],
        condition=IfCondition(LaunchConfiguration('rviz2'))
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz2'))
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        timeout_arg,
        action_timeout_arg,
        slave_address_arg,
        fake_hardware_arg,
        rviz2_arg,
        robotiq_2f_gripper_node,
        rviz2_node,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])