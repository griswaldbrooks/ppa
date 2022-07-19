# rosless
Demo environment for building a rosless C++ package.

## Build
Build a new development image
```shell
mkdir -p ~/.rosless/ccache
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
username@rosless-dev:~/ws$ cmake -S src/rosless/say-hello -B build -DCMAKE_BUILD_TYPE=Release
username@rosless-dev:~/ws$ cmake --build build/
```

## Test
```shell
username@rosless-dev:~/ws cd build
username@rosless-dev:~/ws/build$ ctest
```

## Install
```shell
username@rosless-dev:~/ws$ cmake --install build/ --prefix install/
```

## References
- [Development container documentation](../docs/development-container.md)
