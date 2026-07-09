# Luna ROS 2 Dev Container

This devcontainer provides a complete ROS 2 Humble development environment for the Luna workspace.

## Features

- ROS 2 Humble Desktop Full installation
- VS Code extensions for ROS, Python, and C++ development
- Network access for ROS communication
- GUI application support via X11
- Automatic dependency installation via rosdep

## Usage

1. Open this workspace in VS Code
2. When prompted, click "Reopen in Container" (or run Command Palette: `Remote-Containers: Reopen in Container`)
3. Wait for the container to build and dependencies to install
4. Build the workspace: `colcon build`
5. Source the workspace: `source install/setup.bash`

## Notes

- The ROS_DOMAIN_ID is set to 42 by default. Modify in devcontainer.json if needed.
- Display forwarding is configured for GUI applications
- The container runs as root for simplified device access
