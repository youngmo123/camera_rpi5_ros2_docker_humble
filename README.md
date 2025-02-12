# camera_ros in Docker on a RPi5 via Rasbian Bookworm 64bit lite

An example of how to use `camera_ros` with Raspberry Pi Cameras modules inside an arm64v8/ros:jazzy docker container, running on top of Raspbian OS 64bit Lit (Bookworm).

The example builds and installs raspberrypi's fork of libcamera for support of Raspberry Pi camera modules.

## Requirements

- Raspberry Pi 5 (4BG Model tested)
- Raspberry OS 64Bit Lite (Bookworm)
- Docker

### Tested Cameras
- Raspberry Pi Cam rev1.3 (ov5647)
- Raspberry Pi NoIR Cam (ov5647)

Note: In my testing I did not need to change any parameters in `/boot/firmware/config.txt` on the host OS.

## Setup

Clone this repo and make sure `docker-run.sh` is executable.

```
git clone git@github.com:se1exin/camera_ros-in-docker-rpi5.git

cd camera_ros-in-docker-rpi5

chmod +x docker-run.sh

```

## Build
From the command line on your raspberry pi, run the following to build the container with the tag `camera_ros`:

```
docker build -t camera_ros .
```

## Run
Note: The docker build process adds the file `docker_entrypoint.sh` which sources the required ROS2 `setup.bash` files when the container starts.

From the command line, run the following to start the docker container and the camera_ros node:

```
./docker-run.sh
```


## Modify
The last line of `docker-run.sh` is the command sent to the docker container when it starts. Modify this to - for example - change any ros params when starting the node.

## Notes
You can view the camera stream on another computer using `ros2 run rqt_image_view rqt_image_view`.

Note you may need to add your other IP address as `ROS_STATIC_PEERS` to the docker container to assist ROS2 network communication. E.g. add `-e ROS_STATIC_PEERS=some.internal.ip.address` to the `docker-run.sh` script.  