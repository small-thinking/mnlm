# mnlm
Source code of robotic arm demo (see https://www.bilibili.com/video/BV1ub4y1T7Jt/).


[![IMAGE ALT TEXT HERE](./images/screen.png)](https://www.bilibili.com/video/BV1ub4y1T7Jt/?vd_source=08295b5b4b3c5ece73fb91e3a54d202a)

## Build
```bash
docker build -t mnlm .
```

## Test the docker
Run the docker:
```bash
docker run -it mnlm
```

Run the demo_node_cpp talker
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

In another terminal, run the demo_node_py listener
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```


## Cleanup untagged images
```bash
docker rm $(docker ps -a -q) ; docker images | grep '<none>' | awk '{print $3}' | xargs docker rmi
```