ARG BASE_IMAGE=dtcvc-base

FROM ${BASE_IMAGE} as base

RUN apt-get update && apt-get install -y \
    python3-bson \
    python3-tornado \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros /opt/dtcvc-scorekeeper

WORKDIR /opt/dtcvc-scorekeeper

RUN /bin/bash <<-EOF
    set -e;
    source "/setup.bash";
    rosdep update;
    rosdep install --from-paths src --ignore-src -r --rosdistro $ROS_DISTRO -y;
    colcon build --symlink-install --cmake-args -DSECURITY=ON;
    echo "source /opt/dtcvc-scorekeeper/install/setup.bash" >> /setup.bash
EOF

WORKDIR /

ADD --chmod=755 ./deployment/dtcvc-scorekeeper/run.sh /

ENV PACKAGE_NAME dtcvc_scorekeeper
ENTRYPOINT [ "/run.sh" ]
