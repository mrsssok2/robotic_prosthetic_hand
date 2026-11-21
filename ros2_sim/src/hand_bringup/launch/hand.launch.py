"""Hand simulation: input -> tracker -> controller -> Gazebo Harmonic (ros2_control position controllers).

    ros2 launch hand_bringup hand.launch.py                    # Windows hand_cam.py -> UDP 9870 (default)
    ros2 launch hand_bringup hand.launch.py input:=browser     # optional: browser page http://localhost:8765
    ros2 launch hand_bringup hand.launch.py input:=fake        # animated synthetic hand, no camera
    ros2 launch hand_bringup hand.launch.py input:=none        # only the sim + controller (feed /hand/state yourself)
    ros2 launch hand_bringup hand.launch.py gui:=false         # headless Gazebo
    ros2 launch hand_bringup hand.launch.py rviz:=true
    ros2 launch hand_bringup hand.launch.py grasp:=true        # table with ball / cup / pen, lift joint, contact sensors
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    lc = context.launch_configurations
    gui = lc["gui"] == "true"
    grasp = lc["grasp"] == "true"
    world = "hand_world_grasp.sdf" if grasp else lc["world"]
    bringup = share("hand_bringup")
    urdf = xacro.process_file(
        os.path.join(share("hand_description"), "urdf", "hand.urdf.xacro"),
        mappings={"sim_gazebo": "true", "controllers_yaml": os.path.join(bringup, "config", "controllers.yaml"),
                  "lift": "true" if grasp else "false", **({"base_z": "0.36"} if grasp else {})},
    ).toxml()
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 2 " + os.path.join(bringup, "worlds", world)
                          + ("" if gui else " -s --headless-rendering"),
                          "on_exit_shutdown": "true"}.items())
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": urdf, "use_sim_time": True}])
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "hand"])
    clock = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                 parameters=[{"config_file": os.path.join(bringup, "config", "bridge.yaml")}])
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60", "--switch-timeout", "30"])
    fpc = Node(package="controller_manager", executable="spawner",
               arguments=["finger_position_controller", "--controller-manager-timeout", "60", "--switch-timeout", "30"])
    lift = Node(package="controller_manager", executable="spawner",
                arguments=["lift_controller", "--controller-manager-timeout", "60", "--switch-timeout", "30"])
    extra = []
    if lc["camera"] == "true":  # screenshot world: bridge the fixed camera to /camera/image
        extra.append(Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                          arguments=["/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"]))
    return extra + [gz, rsp, clock, spawn,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[fpc]))] + (
            [RegisterEventHandler(OnProcessExit(target_action=fpc, on_exit=[lift]))] if grasp else [])


def generate_launch_description():
    source = LaunchConfiguration("input")
    from launch.conditions import IfCondition
    from launch.substitutions import PythonExpression

    def is_(value):
        return IfCondition(PythonExpression(["'", source, f"' == '{value}'"]))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo window"),
        DeclareLaunchArgument("input", default_value="udp", description="udp | browser | fake | none"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("grasp", default_value="false", description="grasp world: table + objects + lift joint"),
        DeclareLaunchArgument("world", default_value="hand_world.sdf", description="hand_world.sdf | hand_world_cam.sdf"),
        DeclareLaunchArgument("camera", default_value="false", description="bridge /camera/image (hand_world_cam.sdf)"),
        OpaqueFunction(function=_setup),
        Node(package="hand_tracker", executable="udp_bridge", output="screen", condition=is_("udp")),
        Node(package="hand_tracker", executable="ws_bridge", output="screen", condition=is_("browser")),
        Node(package="hand_tracker", executable="fake_hand", output="screen", condition=is_("fake")),
        Node(package="hand_tracker", executable="tracker_node", output="screen",
             condition=IfCondition(PythonExpression(["'", source, "' in ('browser', 'fake')"]))),
        Node(package="hand_controller", executable="controller_node", output="screen"),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share("hand_description"), "rviz", "hand.rviz")],
             parameters=[{"use_sim_time": True}]),
    ])
