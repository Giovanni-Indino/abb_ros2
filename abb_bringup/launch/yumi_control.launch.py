from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="abb_bringup",
            description="Package with the controller's configuration in config folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="abb_irb14000_support",
            description="Robot description package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="abb_irb14000_moveit_config",
            description="MoveIt config package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="irb14000.urdf.xacro",
            description="URDF/XACRO file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rws_ip",
            default_value="None",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rws_port",
            default_value="80",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "configure_via_rws",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
        )
    )

    # Launch configurations
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    rws_ip = LaunchConfiguration("rws_ip")
    rws_port = LaunchConfiguration("rws_port")
    configure_via_rws = LaunchConfiguration("configure_via_rws")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=", prefix,
            " ",
            "use_fake_hardware:=", use_fake_hardware,
            " ",
            "fake_sensor_commands:=", fake_sensor_commands,
            " ",
            "rws_ip:=", rws_ip,
            " ",
            "rws_port:=", rws_port,
            " ",
            "configure_via_rws:=", configure_via_rws,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Controllers file
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # RViz config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "moveit.rviz"]
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config_file],
        output="log",
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    left_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    right_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )
    ''' 
    left_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "-c", "/controller_manager"],
    )

    right_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "-c", "/controller_manager"],
    )
    '''
    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        left_arm_spawner,
        right_arm_spawner,
        #left_gripper_spawner,
        #right_gripper_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)