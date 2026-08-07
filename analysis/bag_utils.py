"""Shared rosbag helpers for the analysis/ and scripts/ tools (§12).

One place owns the custom-type registration and the bag iteration so the two notebooks and
`scripts/assert_hover.py` cannot drift apart on message names or field access. All three
consumers use `rosbags` (pure Python — no sourced ROS environment needed).

Time base: every helper returns the **bag record timestamp** (nanoseconds, ROS/sim time) for
each message. rosbag2 stamps every message with the same recorder clock, so topics that use
different onboard clocks (PX4's `timestamp` vs the node's `header.stamp`) are aligned by the
record timestamp, and one explicit time base is used throughout. That choice is documented in
`tracking_error_analysis.ipynb` §2.
"""

from __future__ import annotations

import os
from pathlib import Path

# uav_mpc message dependencies, in registration order (SolverDiagnostics is referenced by
# NmpcStatus and must be registered first).
UAV_MPC_MSGS = ("SolverDiagnostics", "NmpcStatus", "AttitudeThrustSetpoint")
PX4_MSGS = ("VehicleLocalPosition", "VehicleAttitude", "VehicleAttitudeSetpoint")
# Standard ROS 2 messages needed by the generic backend.
NAV_MSGS = ("Odometry",)

REPO_ROOT = Path(__file__).resolve().parents[1]


def _rosbags():
    """Import the rosbags modules lazily: bag analysis needs them, --help does not."""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.serde import deserialize_cdr
        from rosbags.typesys import Stores, get_typestore
        from rosbags.typesys import get_types_from_msg
    except ImportError as exc:
        raise RuntimeError(
            "the 'rosbags' package is required for bag analysis "
            "(pip install -r requirements.txt)"
        ) from exc
    return Reader, deserialize_cdr, Stores, get_typestore, get_types_from_msg


def find_px4_msgs_msg_dir() -> Path:
    """Locate the px4_msgs .msg directory without a sourced ROS environment.

    Search order: $PX4_MSGS_DIR, the ROS install trees under /opt/ros, ament_index (only if
    the environment happens to be sourced), and the workspace's src/ tree.
    """
    env = os.environ.get("PX4_MSGS_DIR")
    candidates: list[Path] = []
    if env:
        candidates.append(Path(env))
    for install in Path("/opt/ros").glob("*/share/px4_msgs/msg"):
        candidates.append(install)
    candidates += [
        REPO_ROOT / "px4_msgs" / "msg",
        REPO_ROOT.parent / "px4_msgs" / "msg",
        REPO_ROOT.parent / "src" / "px4_msgs" / "msg",
    ]
    try:  # only works when the ROS environment is sourced
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("px4_msgs")) / "msg")
    except Exception:
        pass
    for cand in candidates:
        if (cand / "VehicleLocalPosition.msg").is_file():
            return cand
    raise FileNotFoundError(
        "cannot locate the px4_msgs message definitions. Set PX4_MSGS_DIR to the "
        "px4_msgs/msg directory, or run from an environment where px4_msgs is installed "
        "(colcon install with px4_msgs in the workspace)."
    )


def build_typestore():
    """Typestore with uav_mpc and px4_msgs custom types registered (ROS 2 distro defaults).

    px4_msgs registration is best-effort: if the message definitions cannot be located
    (e.g. the workspace was built without px4_msgs), it logs a warning and continues
    with only the uav_mpc + nav_msgs types.  .
    """
    import logging

    _log = logging.getLogger(__name__)
    _, _, Stores, get_typestore, get_types_from_msg = _rosbags()
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    def register(msg_dir: Path, prefix: str, names: tuple[str, ...]) -> None:
        for name in names:
            path = msg_dir / f"{name}.msg"
            typestore.register(get_types_from_msg(path.read_text(), f"{prefix}/{name}"))

    register(REPO_ROOT / "uav_mpc" / "msg", "uav_mpc/msg", UAV_MPC_MSGS)

    # px4_msgs: optional.  .
    try:
        register(find_px4_msgs_msg_dir(), "px4_msgs/msg", PX4_MSGS)
    except FileNotFoundError as e:
        _log.warning("px4_msgs not found (%s); PX4-specific topics will not be deserialised", e)

    # Standard nav_msgs types for the generic backend.  .
    try:
        from ament_index_python.packages import get_package_share_directory

        nav_dir = Path(get_package_share_directory("nav_msgs")) / "msg"
        register(nav_dir, "nav_msgs/msg", NAV_MSGS)
    except Exception:
        _log.debug("nav_msgs/Odometry registration skipped (ament_index unavailable)")

    return typestore


def read_topics(bag_path: Path, topics: tuple[str, ...]) -> dict[str, list[tuple[int, object]]]:
    """Read every message of the given topics; returns {topic: [(record_t_ns, msg), ...]}.

    Messages are returned in bag order (approximately chronological). Only the requested
    topics are deserialised; everything else in the bag is ignored.
    """
    Reader, deserialize_cdr, _, _, _ = _rosbags()
    typestore = build_typestore()
    out: dict[str, list[tuple[int, object]]] = {t: [] for t in topics}
    with Reader(bag_path, typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        for conn, timestamp, raw in reader.messages(connections=connections):
            msg = deserialize_cdr(raw, conn.msgtype)
            out[conn.topic].append((timestamp, msg))
    return out


def require_topic(data: dict[str, list], topic: str, bag_path: Path) -> None:
    if not data.get(topic):
        raise RuntimeError(
            f"bag {bag_path} contains no messages on {topic}; is this the right bag? "
            f"(topics found: {sorted(data) or 'none'})"
        )
