FROM ros:humble

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# Install Dependencies for libcamera and ROS2 development
RUN apt update && apt install -y --no-install-recommends gnupg

RUN apt update && apt -y upgrade

RUN apt update && apt install -y --no-install-recommends \
        meson \
	ninja-build \
	pkg-config \
	libyaml-dev \
	python3-yaml \
	python3-ply \
	python3-jinja2 \
	libevent-dev \
	libdrm-dev \
	libcap-dev \
	python3-pip \
	python3-opencv \
	ros-humble-camera-info-manager \
	ros-humble-cv-bridge \
	ros-humble-sensor-msgs \
	ros-humble-std-msgs \
	ros-humble-image-transport \
	ros-humble-image-transport-plugins \
	ros-humble-camera-calibration-parsers \
	ros-humble-camera-calibration \
	ros-humble-ament-cmake-clang-format \
	ros-humble-ament-cmake-copyright \
	ros-humble-ament-cmake-cppcheck \
	ros-humble-ament-cmake-lint-cmake \
	ros-humble-ament-cmake-xmllint \
	ros-humble-ament-cmake-mypy \
	ros-humble-ament-cmake-pycodestyle \
	ros-humble-ament-cmake-pylint \
	ros-humble-ament-lint-auto \
	ros-humble-ament-lint-common \
       && apt-get clean \
       && apt-get autoremove \
       && rm -rf /var/cache/apt/archives/* \
       && rm -rf /var/lib/apt/lists/*

# Install libcamera from source
RUN git clone https://github.com/raspberrypi/libcamera.git && cd libcamera && git checkout 6ddd79b && cd ..
RUN meson setup libcamera/build libcamera/
RUN ninja -C libcamera/build/ install

# Install kmsxx from source
RUN git clone https://github.com/tomba/kmsxx.git
RUN meson setup kmsxx/build kmsxx/
RUN ninja -C kmsxx/build/ install 

# Add the new installations to the python path so that picamera2 can find them
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages:/app/kmsxx/build/py

# Finally install picamera2 using pip
RUN pip3 install picamera2

# Clone and build the camera_ros node
RUN mkdir -p /app/src \
  && cd /app/src \
  && git clone https://github.com/christianrauch/camera_ros.git \
  && source /opt/ros/humble/setup.bash \
  && cd /app \
  && rosdep install -y --from-paths src --ignore-src --rosdistro humble --skip-keys=libcamera \
  && colcon build --event-handlers=console_direct+

COPY docker_entrypoint.sh /app/

ENTRYPOINT ["/app/docker_entrypoint.sh"]
CMD ["bash"]