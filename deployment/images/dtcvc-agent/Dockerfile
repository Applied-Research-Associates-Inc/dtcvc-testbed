ARG BASE_IMAGE=dtcvc-base

FROM ${BASE_IMAGE} AS base

COPY ./ros /opt/dtcvc-competitor
WORKDIR /opt/dtcvc-competitor

# Build competitor
RUN /bin/bash <<-EOF
    source "/setup.bash";
    rosdep update;
    rosdep install --from-paths src --ignore-src -r --rosdistro $ROS_DISTRO -y;
    colcon build --symlink-install --cmake-args -DSECURITY=ON;
    echo "source /opt/dtcvc-competitor/install/setup.bash" >> /setup.bash
EOF
    
WORKDIR /

CMD ["ros2", "launch", "dtcvc_competitor", "dtcvc_competitor.launch.py"]
