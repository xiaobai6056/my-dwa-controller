"""Launch one Waffle RGB world and the optional YOLO detector."""

from pathlib import Path
import os
import sys

from ament_index_python.packages import (
    PackageNotFoundError, get_package_prefix, get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parent
ASSETS = Path(os.environ.get("MYDWA_VISUAL_ASSETS", ROOT)).expanduser().resolve()


def simulation(context):
    model = LaunchConfiguration("model").perform(context)
    v01_run = LaunchConfiguration("v01_run").perform(context)
    explicit_world = LaunchConfiguration("world").perform(context)
    if explicit_world and v01_run:
        raise ValueError("Select world or v01_run")
    if explicit_world:
        world = Path(explicit_world).expanduser().resolve()
    elif v01_run:
        world = Path(v01_run) / "v01.world"
    else:
        world = ASSETS / ".cache" / f"{model}.world"
    urdf = ASSETS / ".cache" / "waffle.urdf"
    if not (world.is_file() and urdf.is_file()):
        raise RuntimeError("Source ROS Humble and run python3 prepare_scene.py first.")

    paths = [str(ASSETS / "models"),
             str(Path(get_package_share_directory("turtlebot3_gazebo")) / "models"),
             "/usr/share/gazebo-11/models"]
    if os.environ.get("GAZEBO_MODEL_PATH"):
        paths.append(os.environ["GAZEBO_MODEL_PATH"])

    server = ExecuteProcess(
        cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_init.so", str(world)],
        output="screen")
    detector = ExecuteProcess(
        cmd=[LaunchConfiguration("python"), "-u", str(ROOT / "person_detector.py"),
             "--ros-args", "-p", ["weights:=", LaunchConfiguration("weights")],
             "-p", ["conf:=", LaunchConfiguration("conf")],
             "-p", ["imgsz:=", LaunchConfiguration("imgsz")],
             "-p", ["device:=", LaunchConfiguration("device")],
             "-p", "use_sim_time:=true"],
        condition=IfCondition(LaunchConfiguration("detector")), output="screen")
    plugin_paths = [str(ASSETS / ".cache" / name)
                    for name in ("v01-gui-build", "v01-gazebo-build")]
    if os.environ.get("GAZEBO_PLUGIN_PATH"):
        plugin_paths.append(os.environ["GAZEBO_PLUGIN_PATH"])
    actions = [
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.pathsep.join(paths)),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.pathsep.join(plugin_paths)),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        server,
        ExecuteProcess(cmd=["gzclient"], condition=IfCondition(LaunchConfiguration("gui")),
                       output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf.read_text(), "use_sim_time": True}],
             output="screen"),
        detector,
    ]
    if v01_run:
        show_viewers = LaunchConfiguration("viewers").perform(context) == "true"
        viewers = image_viewers() if show_viewers else []
        rqt_windows = sum(isinstance(action, ExecuteProcess) for action in viewers)
        controller = ExecuteProcess(
            cmd=[sys.executable, "-u", str(ROOT / "v01_scenario.py"), v01_run,
                 "--ros-args", "-p", "use_sim_time:=true",
                 "-p", f"rqt_windows:={rqt_windows}"], output="screen")
        handlers = [RegisterEventHandler(
            OnProcessExit(target_action=process, on_exit=critical_exit))
                    for process in (server, detector, controller)]
        actions = handlers + actions + [controller, *viewers]
    return actions


def critical_exit(event, context):
    if context.is_shutdown:
        return []
    message = f"{event.process_name} exited before the V01 launch ended (code {event.returncode})"
    if event.returncode:
        raise RuntimeError(message)
    return [EmitEvent(event=Shutdown(reason=message))]


def image_viewers():
    try:
        prefix = Path(get_package_prefix("rqt_image_view"))
        executable = prefix / "lib/rqt_image_view/rqt_image_view"
    except PackageNotFoundError:
        executable = None
    if executable is None or not executable.is_file():
        return [LogInfo(msg="rqt_image_view unavailable. Source the run's ros_env.sh; "
                            "use ros2 run image_tools showimage --ros-args -r image:=TOPIC "
                            "-p reliability:=best_effort -p use_sim_time:=true.")]
    return [
        ExecuteProcess(
            cmd=[str(executable), f"/visual_prototype/{topic}"],
            name=name, output="screen",
        )
        for name, topic in (("raw_image", "camera/image_raw"), ("yolo_image", "annotated"))
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="gems", choices=["gems", "subject"]),
        DeclareLaunchArgument("gui", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("detector", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("python", default_value=str(ASSETS / ".venv/bin/python")),
        DeclareLaunchArgument("weights", default_value=str(ASSETS / "weights/yolov8n.pt")),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("v01_run", default_value=""),
        DeclareLaunchArgument("viewers", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=simulation),
    ])
