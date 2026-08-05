# §2 · Environment

Target platform: **Ubuntu 26.04 LTS + ROS 2 Lyrical Luth** (distro id `lyrical`).

Pinned, and pinned in one place each:

| Component | Version | Pinned in |
| --- | --- | --- |
| Ubuntu | 26.04 LTS | workflow `runs-on` |
| ROS 2 | Lyrical Luth (`lyrical`) | container image, `ROS_DISTRO` |
| Python | 3.14 (system) | `format_check.yml`, `requirements.txt` header |
| PX4 | see `PX4_VERSION` | `.github/workflows/docker_smoke_test.yml` |
| `px4_msgs` | branch/tag matching `PX4_VERSION` **and** built for `lyrical` | `colcon_build.yml` |
| Gazebo | Jetty | `docker_smoke_test.yml` |
| acados | commit in `codegen/ACADOS_COMMIT` | that file |
| CasADi / numpy / … | exact `==` pins | `requirements.txt` |
| Eigen | 3.4 (system) | `package.xml` |

**`px4_msgs` must match the PX4 version.** A mismatched message definition produces no error —
it produces silence, or worse, fields read at the wrong offset. When you touch either version,
touch both.

## Local setup

```bash
mkdir -p ~/ws/src && cd ~/ws/src && git clone <this-repo> uav-mpc
```

Ubuntu 26.04 enforces PEP 668, so pip refuses to install into the system interpreter. Create the
venv with `--system-site-packages` so the ROS 2 Python modules stay importable inside it:

```bash
python3 -m venv --system-site-packages ~/.venvs/uavmpc && . ~/.venvs/uavmpc/bin/activate && pip install -r ~/ws/src/uav-mpc/requirements.txt
```

```bash
export ACADOS_SOURCE_DIR=$HOME/acados && export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

---

## §2.1 Version risk register — read this before writing any CI

Lyrical Luth is a **young distro**, and this repository was scaffolded against it before the
ecosystem settled. Every version in the table above is a forward-looking assumption, not a
verified fact. **Resolve each row below first**; each one fails in a way that wastes hours if
you discover it midway through M6.

| # | Assumption | How it fails | Verify by |
| --- | --- | --- | --- |
| V1 | GitHub provides an `ubuntu-26.04` runner label | workflow never starts: "no runner matching labels" | check the runner-images repo; fall back to `ubuntu-latest` + a `ros:lyrical-*` container, which is what actually determines the build environment anyway |
| V2 | The `ros:lyrical-ros-base` image exists on Docker Hub | `docker pull` fails in every job | pull it locally once |
| V3 | `ros-lyrical-ament-cmake-clang-format` (and cpplint/uncrustify) are published as debs | `format_check.yml` fails at apt | `apt-cache search ros-lyrical-ament` inside the container |
| V4 | `px4_msgs` has a branch supporting Lyrical | builds, then **silently receives nothing** | check the px4_msgs branches; if only older distros are supported, build from the `main` branch and verify a topic actually arrives before assuming success |
| V5 | Gazebo **Jetty** is the Lyrical pairing | version conflicts pulling `gz-*` debs | REP-2000 compatibility table |
| V6 | The pinned PX4 release supports Gazebo Jetty | SITL launches with no vehicle model | PX4 release notes |
| V7 | Python 3.14 wheels exist for casadi/scipy/matplotlib | pip builds from source; CI time explodes | `pip download --only-binary=:all:` for each pin |
| V8 | acados builds against the Ubuntu 26.04 toolchain (newer GCC) | compile errors in blasfeo/hpipm | build it once locally before pinning the commit |
| V9 | `rosbags` parses the Lyrical rosbag2 storage format | analysis notebooks read empty frames | round-trip one recorded bag |

Where a row turns out false, **fix the pin and update the table row to say what is actually
true**. Do not leave a stale assumption in this table — a version register nobody trusts is
worse than no register.

## 2.2 API changes to check against the Lyrical release notes

Two ROS 2 changes since Jazzy that this codebase touches directly. Do not assume either way;
check, then fix at the milestone that needs it, not before.

- **Lifecycle node API** (`rclcpp_lifecycle`) — `nmpc_node.hpp` overrides five transition
  callbacks and uses `CallbackReturn` from `LifecycleNodeInterface`. If the signatures or the
  header path moved, fix them at M6.
- **Launch lifecycle events** (`EmitEvent` / `OnStateTransition`, see
  [08_LAUNCH.md §8.1](08_LAUNCH.md)) — the event API has been reorganised across distros
  before. If it moved, the *pattern* still stands: event-driven sequencing, never a
  `TimerAction` race.

## 2.3 PEP 668

Ubuntu 26.04 marks the system interpreter externally-managed, so `pip install` into it is
refused outright.

**The decision for this repository:** a venv created with `--system-site-packages` (so `rclpy`
and the other ROS 2 Python modules remain importable), with its `bin/` prepended to `PATH`.

**Do not use `--break-system-packages`.** It works right up until it corrupts a
rosdep-installed package, and then the failure presents as a ROS bug and costs a day to trace.

The `format_check.yml` python job is the one exception — it uses `actions/setup-python`, so
PEP 668 never enters the picture there.
