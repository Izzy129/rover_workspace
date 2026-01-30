# ros2_ws_template

This repository serves as a template for all ROS2 workspaces to be created for the Rice Robotics Club. It features a basic scaffolding for the workspace, including src/ directory and a .gitignore file to block build directories from being uploaded to the repository. It also features a .devcontainer setup that allows running packages in a Docker container that can be spun up by a simple command in VSCode.

## Running Devcontainer

1. Make sure you have Docker Engine installed. On Windows and MacOS, this is done by installing [Docker Desktop](https://docs.docker.com/desktop/). On Linux, you can either install Docker Desktop or just [Docker Engine](https://docs.docker.com/engine/install/), which provides only the `docker` cli.
2. In VSCode, install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
3. Clone the desired ROS2 workspace repository.
4. Open the repository folder in VSCode. The Dev Containers extension should then recognize that there is a .devcontainer/ directory in the workspace folder. On the bottom right, click the button that says "Reopen in Container". This will then build the Docker container, and then reopen the project folder inside that container. From here, you can then access the VSCode terminal as if you are running Ubuntu with ROS2 installed.
5. You can test this by running the following command in the terminal:
```bash
sudo apt install ros-$ROS_DISTRO-rviz2 -y
source /opt/ros/$ROS_DISTRO/setup.bash
rviz2
```
