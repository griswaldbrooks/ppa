# rosful
Demo environment for consuming a rosless package in a ros node.

# Build
Build a new development image
```
mkdir -p ~/.rosful/ccache
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml run development
```
Build the repository in the container
```
username@rosful-dev:~/ws$ colcon build
```

# Test
```shell
username@rosful-dev:~/ws$ colcon build
username@rosful-dev:~/ws$ source install/setup.bash
username@rosful-dev:~/ws$ ros2 run rosful naive
```

## References
- [Development container documentation](../docs/development-container.md)
