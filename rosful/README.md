# rosful
Demo environment for consuming a rosless package in a ros node.

# Build
Build a new development image
```shell
mkdir -p ~/.rosful/ccache
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
username@rosful-dev:~/ws$ colcon build
```

# Test
```shell
username@rosful-dev:~/ws$ colcon build
username@rosful-dev:~/ws$ source install/setup.bash
username@rosful-dev:~/ws$ ros2 run rosful node
[INFO] [1658266019.972607988] [speaker]: I want tacos
```

## References
- [Development container documentation](../docs/development-container.md)
