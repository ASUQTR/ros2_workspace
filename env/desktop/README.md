# Desktop Environment

This environment lets you develop and test ROS 2 code inside a Docker container on any x86_64 machine (Ubuntu 20.04+, macOS, WSL2). It does **not** include Jetson hardware drivers (Bar30, GPIO, PCA9685) — only the VectorNav IMU via USB is supported.

> [!NOTE]
> Make sure `requirements_desktop.txt` and `underlay.repos` are up to date before building the Docker image.

## Build the image (first time only)

From the **repo root**:

```bash
./start-desktop.sh
```

The script automatically checks whether the `ros2-humble` image exists. If not, it calls `env/desktop/setup.sh` to build it (~20-30 min depending on your connection), then starts the container.

## Open additional terminals

```bash
docker exec -it ros2-desktop /bin/bash
```

ROS 2 sources and ASUQTR aliases are loaded automatically on every terminal open.

## Available aliases

Once inside the container, type `asuqtr-help` to list all shortcuts:

```
asuqtr-build              Build & source the workspace
asuqtr-launch             Launch the desktop system (IMU only)
asuqtr-motors             Thruster commands in Newtons
asuqtr-pos                Estimated XYZ position
asuqtr-target x y z r p y  Send a target pose to the controller
...
```
