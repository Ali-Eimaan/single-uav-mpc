"""SKELETON — the one-command demo. See .deepseek/08_LAUNCH.md §8.2.

Brings up, in dependency order:
    1. PX4 SITL + Gazebo Jetty (gz_x500 model)
    2. MicroXRCEAgent udp4 -p 8888
    3. the NMPC lifecycle node (via nmpc_only.launch.py)
    4. RViz2 with rviz/nmpc.rviz

    ros2 launch uav_mpc sitl.launch.py world:=default headless:=false

This file is the "clone and fly in 10 minutes" promise from the README. If it is fragile,
the repo fails its primary purpose — prefer explicit checks and loud error messages over
clever automation.
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    TODO(deepseek): implement.

    Declared arguments:
        px4_dir       default os.environ.get("PX4_DIR", "~/PX4-Autopilot") — fail loudly with an
                      actionable message if the directory does not exist
        model         default "gz_x500"
        world         default "default"
        headless      bool, default false  -> sets HEADLESS=1 in the PX4 environment
        agent_port    default 8888
        rviz          bool, default true
        auto_arm      bool, default true   (SITL only — hardware.launch.py defaults to false)
        trajectory    default "hover"      -> passed through to the NMPC node

    Processes / includes:
        ExecuteProcess PX4:
            cmd = [px4_dir + "/build/px4_sitl_default/bin/px4"] with additional_env
                  {"PX4_SYS_AUTOSTART": "4001", "PX4_GZ_MODEL": model, "PX4_GZ_WORLD": world,
                   "HEADLESS": ...}
            cwd = px4_dir/build/px4_sitl_default
        ExecuteProcess MicroXRCEAgent: ["MicroXRCEAgent", "udp4", "-p", str(agent_port)]
        TimerAction(2.0) around the agent so PX4 has booted first, and
        RegisterEventHandler(OnProcessStart(px4)) to start it deterministically where possible.
        IncludeLaunchDescription(nmpc_only.launch.py) delayed until the agent is up
            (TimerAction 5.0 is acceptable here; document why).
        Node rviz2 with -d <share>/rviz/nmpc.rviz, condition IfCondition(rviz).

    Also:
        - apply config/px4_overrides.yaml to the running SITL instance. The pragmatic route is
          to write a PX4 startup snippet into the ROMFS extras file; the guide (§8.2) spells
          out the exact mechanism. Do NOT silently skip this — the thrust map depends on it.
        - on_shutdown handler that SIGINTs PX4 and the agent so a Ctrl-C leaves no orphans.
    """
    raise NotImplementedError("sitl.launch.py not implemented")
