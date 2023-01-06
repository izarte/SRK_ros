# Srk ros packages

Ros packages to succefully connect unity with ros and aruco detection [SRK]

## Prerequisites
- Docker

## Installation
Update submodules
```sh
git submodule init
git submodule update
```

### Docker
To launch docker container, use this command into repository:
```sh
docker run -v <FOLDER_PATH>/catkin_ws:/catkin_ws --volume=/tmp/.X11-unix:/tmp/.X11-unix --volume=src/srk/models:/root/.gazebo --env="DISPLAY" --net=host -it --rm --name ros_noetic inigo183/ros_noetic:latest /bin/bash
```

First of all we have to compile the packages, to di so in docker container execute:

```sh
cd catkin_make
catkin_make
```

## Start Up

Terminal 1 (same as previous installation)
in catkin_ws
```sh
source devel/setup.bash
roscore &
rosparam set ROS_IP <YOUR_PC_IP>
rosrun ros_tcp_endpoint default_server_endpoint.py
```

Terminal 2
```sh
docker exec -it ros_noetic /ros_entrypoint.sh /bin/bash
```
```sh
cd catkin_ws
source devel/setup.bash
rosrun srk_move aruco.py
```


[SRK]: https://github.com/izarte/SRK

