ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO-ros-base
ENV DEBIAN_FRONTEND=noninteractive PIP_BREAK_SYSTEM_PACKAGES=1
WORKDIR /root/

# Install general packages (including foxglove)
RUN rm /var/lib/dpkg/info/libc-bin.* \
    && apt-get clean \
    && apt-get update \
    && apt-get install libc-bin \
    && apt-get install -q -y --no-install-recommends \
    #  libasio-dev libgeographic-dev geographiclib-tools \
    git tmux nano nginx wget libboost-all-dev\
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-eigen-stl-containers \
    ros-${ROS_DISTRO}-mavlink \
    python3-dev python3-pip python3-click python3-scipy python3-venv \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

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
    

# Build ROS2 workspace with remaining packages
COPY ros2_ws /root/ros2_ws
RUN rm -rf /root/ros2_ws/src/mavros
RUN cd /root/ros2_ws/ \
    && pip3 install --no-cache-dir -r src/mavros_control/requirements.txt \
    # && pip3 install --no-cache-dir -r src/sonar3d/src/sonar3d/sonar3d/api/requirements.txt \
    && git clone https://github.com/ptrmu/ros2_shared.git --depth 1 src/ros2_shared \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys nlohmann_json \ 
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && ros2 run mavros install_geographiclib_datasets.sh 

RUN cd /root/ros2_ws/ \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /ros_entrypoint.sh" >> ~/.bashrc \
    && echo "source /root/ros2_ws/install/setup.sh" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc \
    && echo "alias cw='cd /root/ros2_ws'" >> ~/.bashrc \
    && echo "alias cs='cd /root/ros2_ws/src'" >> ~/.bashrc \
    && echo "alias cb='cd /root/ros2_ws && colcon build'" >> ~/.bashrc
    # RUN apt-get update && apt-get install dos2unix -y
    
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
LABEL version="0.2.1"
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
LABEL readme="https://raw.githubusercontent.com/Projeto-Voris/blueos-ros2/main/README.md"
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
