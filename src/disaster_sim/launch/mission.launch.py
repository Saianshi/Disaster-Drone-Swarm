#!/usr/bin/env python3
"""One command runs the whole demo:

    ros2 launch disaster_sim mission.launch.py

Brings up Gazebo Classic (disaster_city world), spawns six camera drones next
to the 5G van, starts the kinematic swarm controller, RViz2 with the mission
view, and the scripted mission node.

Arguments (all optional; typos such as `climb:=2.0~` are tolerated - the
number/boolean is extracted and a warning is printed instead of the whole
launch crashing):
    gui:=false      headless Gazebo (no gzclient window)        default true
    rviz:=false     don't start RViz2                            default true
    hold:=false     shut everything down when the mission ends   default true
                    (true = stay open after the alert until Ctrl-C)
    speed:=N        drone max horizontal speed, m/s              default 4.0
    climb:=N        drone max climb rate, m/s                    default 2.0
    world_wait:=N   seconds to let the world load before spawns  default 45
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction,
                            OpaqueFunction, RegisterEventHandler, EmitEvent,
                            SetEnvironmentVariable, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

PKG = "disaster_sim"
SHARE = get_package_share_directory(PKG)

# ns -> (x, y, z, yaw, marker r, g, b)
# spawn slots: 3x2 grid just south of the 5G van (~125, -8), 3 m apart, on
# the ground (z 0.15). yaw 3.0 = facing west toward the building.
DRONES = {
    "drone_0": (121.0, -13.0, 0.15, 3.0, 0.95, 0.15, 0.10),
    "drone_1": (124.0, -13.0, 0.15, 3.0, 0.98, 0.55, 0.05),
    "drone_2": (127.0, -13.0, 0.15, 3.0, 0.95, 0.90, 0.12),
    "drone_3": (121.0, -16.0, 0.15, 3.0, 0.15, 0.85, 0.22),
    "drone_4": (124.0, -16.0, 0.15, 3.0, 0.12, 0.78, 0.92),
    "drone_5": (127.0, -16.0, 0.15, 3.0, 0.90, 0.18, 0.80),
}
NAMES = list(DRONES)


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _num(context, name, default, warn):
    raw = _arg(context, name)
    m = re.search(r"-?\d+(?:\.\d+)?", raw)
    val = float(m.group()) if m else default
    if not m or m.group() != raw.strip():
        warn.append(f"argument {name}:='{raw}' is not a clean number -> using {val}")
    return val


def _bool(context, name, default, warn):
    raw = _arg(context, name).strip().lower()
    if raw in ("true", "1", "yes", "on"):
        return True
    if raw in ("false", "0", "no", "off"):
        return False
    val = raw.startswith(("t", "y", "1")) if raw else default
    warn.append(f"argument {name}:='{raw}' is not true/false -> using {val}")
    return val


def _setup(context, *args, **kwargs):
    warn = []
    gui = _bool(context, "gui", True, warn)
    use_rviz = _bool(context, "rviz", True, warn)
    hold = _bool(context, "hold", True, warn)
    speed = _num(context, "speed", 4.0, warn)
    climb = _num(context, "climb", 2.0, warn)
    base = _num(context, "world_wait", 45.0, warn)

    out = [LogInfo(msg=f"disaster_sim WARNING: {w}") for w in warn]
    out.append(LogInfo(msg=(f"disaster_sim: gui={gui} rviz={use_rviz} hold={hold} "
                            f"speed={speed} climb={climb} world_wait={base}")))

    world = os.path.join(SHARE, "worlds", "disaster_city.world")
    rviz_cfg = os.path.join(SHARE, "rviz", "mission.rviz")

    out.append(ExecuteProcess(
        name="gzserver",
        cmd=["gzserver", "--verbose", world,
             "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"],
        output="screen"))
    if gui:
        out.append(ExecuteProcess(name="gzclient", cmd=["gzclient"], output="log"))
    if use_rviz:
        out.append(TimerAction(period=6.0, actions=[Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", rviz_cfg], output="log")]))

    # -- drones: one sdf per drone from the template, spawned after the world loads
    tmpl = open(os.path.join(SHARE, "description", "drone.sdf.template")).read()
    gen = "/tmp/disaster_sim_gen"
    os.makedirs(gen, exist_ok=True)
    for i, (ns, (x, y, z, yaw, r, g, b)) in enumerate(DRONES.items()):
        sdf = (tmpl.replace("__NS__", ns)
                   .replace("__X__", str(x)).replace("__Y__", str(y))
                   .replace("__Z__", str(z)).replace("__YAW__", str(yaw))
                   .replace("__R__", str(r)).replace("__G__", str(g))
                   .replace("__B__", str(b)))
        path = os.path.join(gen, f"{ns}.sdf")
        with open(path, "w") as f:
            f.write(sdf)
        out.append(TimerAction(period=base + 1.8 * i, actions=[Node(
            package="gazebo_ros", executable="spawn_entity.py",
            name=f"spawn_{ns}", output="log",
            arguments=["-entity", ns, "-file", path,
                       "-x", str(x), "-y", str(y), "-z", str(z),
                       "-timeout", "150.0"])]))

    last_spawn = base + 1.8 * (len(DRONES) - 1)

    controller = Node(
        package=PKG, executable="swarm_controller.py", name="swarm_controller",
        output="screen",
        parameters=[{
            "drones": NAMES,
            "init_x": [DRONES[n][0] for n in NAMES],
            "init_y": [DRONES[n][1] for n in NAMES],
            "init_z": [DRONES[n][2] for n in NAMES],
            "init_yaw": [DRONES[n][3] for n in NAMES],
            "max_speed": speed,
            "max_climb": climb,
        }])
    mission = Node(
        package=PKG, executable="mission_node.py", name="mission_node",
        output="screen", parameters=[{"hold": hold}])

    out.append(TimerAction(period=last_spawn + 3.0, actions=[controller]))
    out.append(TimerAction(period=last_spawn + 12.0, actions=[mission]))
    out.append(RegisterEventHandler(OnProcessExit(
        target_action=mission,
        on_exit=[LogInfo(msg="mission_node finished - shutting the demo down"),
                 EmitEvent(event=Shutdown(reason="mission complete"))])))
    return out


def generate_launch_description():
    set_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.path.join(SHARE, "models") + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""))
    set_res_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH",
        SHARE + ":" + os.environ.get("GAZEBO_RESOURCE_PATH", ""))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("hold", default_value="true",
                              description="stay open after the alert until Ctrl-C"),
        DeclareLaunchArgument("speed", default_value="4.0",
                              description="drone max horizontal speed (m/s)"),
        DeclareLaunchArgument("climb", default_value="2.0",
                              description="drone max climb rate (m/s)"),
        DeclareLaunchArgument("world_wait", default_value="45"),
        set_model_path,
        set_res_path,
        # every node runs on Gazebo sim time so RViz can sync camera + TF
        SetParameter(name="use_sim_time", value=True),
        OpaqueFunction(function=_setup),
    ])
