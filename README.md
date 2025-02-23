# farm_ws



## Cloning the Repository

To get started, clone the repository:
```sh
git clone https://github.com/TKUwengkunduo/farm_ws.git
```

## Building and Running Docker Container

Navigate to the Docker setup directory and execute the build and run scripts:
```sh
cd farm_ws/docker/ubuntu
./build.sh
./run.sh
```

## Building ROS2 Workspace

Once inside the Docker container, build the ROS2 workspace:
```sh
colcon build
source install/setup.bash
```

