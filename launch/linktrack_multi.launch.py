from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Spawn one linktrack_node per port, each in its own namespace.

    The topics in linktrack_node are relative names, so a namespace per module
    keeps them apart (/uwb0/nlink_linktrack_nodeframe5, /uwb1/..., ...).
    One process per module is required: the publisher map in linktrackinit.cpp
    is a namespace-level global, so two Init objects in one process would share
    it.
    """
    ports = [p for p in LaunchConfiguration('port_names').perform(context).split(',') if p]
    prefix = LaunchConfiguration('ns_prefix').perform(context)
    baud_rate = int(LaunchConfiguration('baud_rate').perform(context))

    return [
        Node(
            package='nlink_parser2',
            executable='linktrack_node',
            name='linktrack',
            namespace=f'/{prefix}{index}',
            output='screen',
            parameters=[{'port_name': port, 'baud_rate': baud_rate}],
        )
        for index, port in enumerate(ports)
    ]


def generate_launch_description():

    port_names_arg = DeclareLaunchArgument(
        'port_names',
        default_value='/dev/ttyUSB0,/dev/ttyUSB1',
        description='Comma-separated serial ports, one per module. Prefer the stable '
                    '/dev/serial/by-id/... paths, since ttyUSBn renumbers across replugs.'
    )

    ns_prefix_arg = DeclareLaunchArgument(
        'ns_prefix',
        default_value='uwb',
        description='Namespace prefix; nodes become /uwb0/linktrack, /uwb1/linktrack, ...'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Serial baud rate, applied to every module'
    )

    return LaunchDescription([
        port_names_arg,
        ns_prefix_arg,
        baud_rate_arg,
        OpaqueFunction(function=launch_setup),
    ])
