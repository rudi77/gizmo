"""Launch Gizmo in Gazebo Harmonic + spawn ros2_control controllers.

Avoids two known traps:
1. Modern Gazebo does NOT expand ROS-style $(find pkg) inside <plugin>/<parameters>.
   We resolve the controllers YAML path at launch time and pass it into xacro
   as a mapping, so the URDF arrives at Gazebo with an absolute path.

2. Spawner timing: chaining via OnProcessExit fires the next spawner the moment
   its target exits, but the target ('create') exits before controller_manager
   has actually advertised its services inside the running sim. We use
   TimerAction with conservative delays plus --controller-manager-timeout so
   the spawners wait for the manager to come up.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = Path(get_package_share_directory("gizmo_description"))
    xacro_file = desc_share / "urdf" / "gizmo.urdf.xacro"
    controllers_yaml = desc_share / "config" / "gizmo_controllers.yaml"

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", str(xacro_file),
                " controllers_yaml:=", str(controllers_yaml),
            ]),
            value_type=str,
        ),
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("ros_gz_sim"))
                / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    # Spawn height matches the symmetric stand pose in gizmo_gait_node:
    # body_link sits at z = 0.17 m, which puts the foot spheres just on
    # the ground when hips=0 and knees=-0.6 rad.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "gizmo", "-z", "0.17"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gizmo_joint_trajectory_controller",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    delayed_spawn = TimerAction(period=4.0, actions=[spawn])
    delayed_jsb = TimerAction(period=8.0, actions=[jsb_spawner])
    delayed_jtc = TimerAction(period=12.0, actions=[jtc_spawner])

    return LaunchDescription([
        gz_sim,
        rsp,
        clock_bridge,
        delayed_spawn,
        delayed_jsb,
        delayed_jtc,
    ])
