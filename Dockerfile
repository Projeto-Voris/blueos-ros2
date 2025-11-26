ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO-ros-base
ENV DEBIAN_FRONTEND=noninteractive PIP_BREAK_SYSTEM_PACKAGES=1
WORKDIR /root/

RUN echo "deb http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse" >> /etc/apt/sources.list

# Install general packages (including foxglove)
RUN rm /var/lib/dpkg/info/libc-bin.* \
    && apt-get clean \
    && apt-get update \
    && apt-get install libc-bin \
    && apt-get install -q -y --no-install-recommends \
    libboost-all-dev libasio-dev libgeographic-dev geographiclib-tools \
    git tmux nano nginx wget netcat \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-eigen-stl-containers \
    ros-${ROS_DISTRO}-mavlink \
    python3-dev python3-pip python3-click python3-scipy python3-venv \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install pip3 and bluerobotics-ping
RUN pip3 install --no-cache-dir setuptools -U \
    && pip3 install --no-cache-dir bluerobotics-ping

# Install gscam2 deps
WORKDIR /root/
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
    libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
    gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
    
# Install ping-python from source (Newer commits don't seem to work with the ping1d)
RUN cd /root/ \
    && git clone https://github.com/bluerobotics/ping-python.git -b deployment \
    && cd ping-python \
    && git checkout 3d41ddd \
    && python3 setup.py install --user

# Install packages by source code
# COPY ros2_ws /root/ros2_ws
# WORKDIR /root/ros2_ws/src
# RUN git config --global url."https://github.com/".insteadOf "git@github.com:" && \
#     git clone --recurse-submodules https://github.com/JetSeaAI/ping360_sonar.git && \
#     git clone https://github.com/clydemcqueen/gscam2.git && \
#     git clone https://github.com/itskalvik/bluerobotics_sonar.git && \
#     git clone https://github.com/itskalvik/sonoptix_sonar.git && \
#     git clone https://github.com/Projeto-Voris/voris_bringup.git && \
#     git clone https://github.com/mavlink/mavros.git -b ros2 && \
#     cd mavros && git checkout e7a3e40e && cd ..
  
    
# ----------- Changes on source code stay here ----------- #
    
# COPY files/imu.cpp.modificado /root/ros2_ws/src/mavros/mavros/src/plugins/imu.cpp
# COPY files/ping360_files/ping360_node.cpp.modificado /root/ros2_ws/src/ping360_sonar/ping360_sonar/src/ping360_node.cpp
# COPY files/gscam2_files/params.yaml.modificado /root/ros2_ws/src/gscam2/cfg/params.yaml
# COPY files/ping360_files/ping360_bringup.launch.py.modificado /root/ros2_ws/src/ping360_sonar/ping360_sonar/launch/ping360_bringup.launch.py
    
# -------------------------------------------------------- #
    

# Build ROS2 workspace with remaining packages
WORKDIR /root/ros2_ws/
RUN pip3 install --no-cache-dir -r src/mavros_control/requirements.txt \
    && git clone https://github.com/ptrmu/ros2_shared.git --depth 1 src/ros2_shared \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && . /root/ros2_ws/install/setup.sh \
    && ros2 run mavros install_geographiclib_datasets.sh \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /ros_entrypoint.sh" >> ~/.bashrc \
    && echo "source /root/ros2_ws/install/setup.sh " >> ~/.bashrc

# Setup ttyd for web terminal interface
ADD files/install-ttyd.sh /install-ttyd.sh
RUN bash /install-ttyd.sh && rm /install-ttyd.sh

# Copy configuration files
COPY files/nginx.conf /etc/nginx/nginx.conf
COPY files/index.html /usr/share/ttyd/index.html

# Copy start script and other files
RUN mkdir -p /site
COPY files/register_service /site/register_service
COPY files/start.sh /start.sh

# Add docker configuration
LABEL version="1.0.5"
LABEL permissions='{\
  "NetworkMode": "host",\
  "HostConfig": {\
    "Binds": [\
      "/dev:/dev:rw",\
      "/usr/blueos/extensions/ros2/:/root/persistent_ws/:rw"\
    ],\
    "Privileged": true,\
    "NetworkMode": "host",\
    "CpuQuota": 200000,\
    "CpuPeriod": 100000,\
    "Memory": 1097152000\
  },\
  "Env": [\
  ]\
}'
LABEL authors='[\
  {\
    "name": "Kalvik Jakkala",\
    "email": "itskalvik@gmail.com"\
  },\
  {\
    "name": "Miguel Soria",\
    "email": "miguel.luz@labmetro.ufsc.br"\
  }\
]'
LABEL company='{\
  "about": "",\
  "name": "VORIS / ItsKalvik",\
  "email": "projeto.voris@labmetro.ufsc.br"\
}'
LABEL readme="https://raw.githubusercontent.com/miguelslz/blueos-ros2/main/README.md"
LABEL type="other"
LABEL tags='[\
  "ros2",\
  "sonar",\
  "camera",\
  "foxglove",\
  "navigation",\
  "mapping",\
  "data-collection",\
  "communication",\
  "interaction",\
  "positioning"\
]'

# Keep bash alive even if there is an error
RUN echo "set +e" >> ~/.bashrc
ENTRYPOINT [ "/start.sh" ]
