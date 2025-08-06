FROM ros:humble

SHELL ["/bin/bash", "-c"]

# 기본 패키지 설치
RUN apt update && apt install -y --no-install-recommends \
    gnupg \
    git \
    python3-pip \
    python3-jinja2 \
    python3-yaml \
    python3-ply \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff-dev \
    pybind11-dev \
    meson \
    cmake \
    ninja-build \
    libglib2.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libevent-dev \
    libdrm-dev \
    libcap-dev \
    python3-opencv \
 && apt-get clean \
 && apt-get autoremove \
 && rm -rf /var/cache/apt/archives/* \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# =============================
# libcamera 설치 (라즈베리파이 공식 소스)
# =============================
RUN git clone https://github.com/raspberrypi/libcamera.git \
 && cd libcamera \
 && git checkout 6ddd79b \
 && meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled \
 && ninja -C build install \
 && ldconfig

# =============================
# camera_ros 패키지 빌드
# =============================
RUN mkdir -p /app/src \
 && cd /app/src \
 && git clone https://github.com/christianrauch/camera_ros.git

# ROS2 빌드 준비
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && cd /app \
 && rosdep update \
 && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera \
 && . /opt/ros/$ROS_DISTRO/setup.bash \
 && colcon build --event-handlers=console_direct+

# 엔트리포인트 스크립트 복사
COPY docker_entrypoint.sh /app/
RUN chmod +x /app/docker_entrypoint.sh

# 환경변수 설정
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages

ENTRYPOINT ["/app/docker_entrypoint.sh"]
CMD ["bash"]
