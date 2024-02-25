

# Development

## Project structure
```

mnlm
|-> mnlm
|     |--> client
|          |--> gpt_control
|               |--> assistant.py  # voice based UI start program
|               |--> command_indexer.py  # script to index commands into the knowledge base
|          |--> knowledge  # the knowledge base
|          |--> dummy_robot_arm_server.py  # dummy server for testing purpose so we don't need to start ROS2 server
|     |--> robot
|          |--> robot_arm_ws  # ROS2 workspace
|               |--> robot_arm  # ROS2 package
|                    |--> launch  # launch file
|                    |--> config  # configuration file, includes the ros2 control configuration
|                    |--> src  # source code
|                    |--> models  # robot model, includes the xacro and sdf files
|                    |--> setup.py  # setup file for the package
|                    |--> package.xml  # package file
|-> docker-compose.yml  # docker compose file
```

## Cleanup untagged images
```bash
docker rm $(docker ps -a -q) ; docker images | grep '<none>' | awk '{print $3}' | xargs docker rmi
```
