# Repository Guidelines

## Project Structure & Module Organization
- Catkin workspace root; `run.sh --init` populates `src/` via `west` (see `west.yml`) with `jetbot_diff_drive` (JetBot sim/control), `turtlebot3_ddpg_collision_avoidance` (DDPG agent, Gazebo worlds/launch), `turtlebot3` + `turtlebot3_msgs` (base stacks), and simulation plugins `realsense_gazebo_plugin` + `hector_gazebo_plugins`.
- `Dockerfile` defines the ROS Noetic + Python 3.7 image. `run.sh` orchestrates container lifecycle, dependency install, and launch. `ddpg_env/` is the Python 3.7 virtualenv inside the container. Build outputs live in `build/` and `devel/` (generated inside the container).
- Keep package-specific assets (models, meshes, config YAML, launch files) under their respective package directories; avoid host-level paths in code or launch files.

## Build, Test, and Development Commands
- `./run.sh --init` — builds the Docker image if needed, initializes/updates west projects, installs Python deps (TF 1.15/Keras 2.3.1), resolves ROS deps via `rosdep`, and runs `catkin_make`.
- `./run.sh --run` — starts the container, launches Gazebo via `turtlebot_ddpg/main.launch`, then runs the DDPG agent script.
- Inside the container: `west update` to sync repos, `catkin_make` for incremental builds, `catkin_make run_tests` for ROS/rostest/gtest targets. Always `source /opt/ros/noetic/setup.bash && source devel/setup.bash` before manual ROS commands.

## Coding Style & Naming Conventions
- Python (agent/training scripts): PEP 8, 4-space indents, snake_case modules/functions, UPPER_SNAKE constants; group imports stdlib/third-party/local. Prefer explicit ROS parameter names and avoid wildcard imports.
- ROS C++ nodes: follow existing ROS style (brace on same line, 2-space indents if matching upstream), `PascalCase` classes, `camelCase` variables/functions. Launch file names stay descriptive (`main.launch`, `bringup.launch`), with package-relative paths for meshes/models.
- Namespace/parameter names should be lowercase without hyphens; keep configuration YAML next to the launch that consumes it.

## Testing Guidelines
- Primary fast check: run inside the container `catkin_make run_tests` to execute package tests/rostests. New packages should wire tests via `catkin_add_gtest` or `rostest` in their `CMakeLists.txt`.
- For agent or reward logic changes, validate end-to-end with `./run.sh --run`; confirm Gazebo starts cleanly and the agent reaches stable rewards. Log metrics to a package-local `logs/` or `results/` directory (create if missing).
- Name tests after the node or script under test (e.g., `test_laser_scan_handler.cpp`, `test_reward_shaping.test`) and keep fixtures within the same package.

## Commit & Pull Request Guidelines
- Commit messages: short, imperative (`Add lidar preprocessing`, `Fix DDPG reward reset`); keep one logical change per commit.
- PRs should explain the issue, approach, and validation commands (init/run/tests). Link issues and mention changes to `west.yml`, Docker image deps, or new environment variables.
- Include screenshots or terminal snippets when modifying worlds, launch behavior, or training output. Avoid host-only dependencies; keep everything reproducible inside the container.

## Environment & Security Notes
- Develop inside the Docker container; do not rely on host-installed Python/ROS packages. Activate `/root/catkin_ws/ddpg_env` for agent scripts.
- After editing `west.yml`, rerun `./run.sh --init` or `west update` in the container to sync sources; do not commit `.west` or build artifacts.
- The container uses host networking and X11; ensure `$DISPLAY` is set and you are comfortable with `--net=host` when sharing usage instructions.
