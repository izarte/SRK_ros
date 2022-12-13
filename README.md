# Srk ros packages

Ros packages to succefully connect unity with ros and logic for SRK (link)

## Prerequisites
- Docker

## Installation
Clone repository and update submodules
```sh
git clone https://github.com/izarte/SRK_ros
git submodule init
git submodule update
```

### Docker
docker container to initialize packages

in catkin_ws

```sh
catkin_make
```

We have to copy models for specific path to use gazebo simulation with arucos, in catkin_ws folder

```sh
cp -r src/srk/models /root/.gazebo/
```

## Start Up

Terminal 1
```sh
docker run -v <FOLDER_PATH>/catkin_ws:/catkin_ws --volume=/tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri:/dev/dri --env="DISPLAY" --net=host -it --rm --name ros_noetic inigo183/ros_noetic:odom /bin/bash
```

```sh
cd catkin_ws
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
rosrun srk_move move.py
```
## Unity Steps
Open project and set your ip to Ros connections. Robotics->Ros Settings and in Ros_Connection object.

Choose message path, Robotics->Generate Ros Messages->Ros Message path and select srk_move folder. Click in build msg and srv.
