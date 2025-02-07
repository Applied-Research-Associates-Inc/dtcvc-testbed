ARG ROS_DISTRO=iron

FROM osrf/ros:${ROS_DISTRO}-simulation

ARG ROS_DISTRO

RUN apt-get update && apt-get install -y \
    ros-rolling-derived-object-msgs \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-demo-nodes-py \
    ros-$ROS_DISTRO-demo-nodes-cpp \
    libwebsocketpp-dev \
    nlohmann-json3-dev \
    libasio-dev \
    pip \
    net-tools \
    iputils-ping \
    dnsutils \
    netcat-traditional \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Install the CARLA Python library bundled with the simulator
RUN --mount=from=external-packages,source=PythonAPI/carla/dist,target=./dist \
    pip3 install \
    ./dist/carla-0.10.0-cp310-cp310-linux_x86_64.whl

# Add initial setup script

COPY <<EOF /setup.bash
#!/bin/bash

source "/opt/ros/${ROS_DISTRO}/setup.bash"
EOF

# Build scorekeeper message types so they're available to both competitor and competition images downstream

COPY ./ros/src/dtcvc_scorekeeper_types /opt/dtcvc_scorekeeper_types/src/dtcvc_scorekeeper_types
WORKDIR /opt/dtcvc_scorekeeper_types
RUN /bin/bash <<-EOF
    source "/setup.bash";
    rosdep update;
    rosdep install --from-paths src --ignore-src -r -y;
    colcon build --symlink-install --cmake-args -DSECURITY=ON;
    echo "source /opt/dtcvc_scorekeeper_types/install/setup.bash" >> /setup.bash
EOF

# The triage scoring library is independent of ROS so it has to be added to the path manually
COPY ./lib /opt/dtcvc-scorekeeper/lib
RUN pip3 install -r /opt/dtcvc-scorekeeper/lib/dtcvc-scoring/requirements.txt
ENV PYTHONPATH "${PYTHONPATH}:/opt/dtcvc-scorekeeper/lib/dtcvc-scoring"
ENV DTCVC_SCORING_LIB_PATH "/opt/dtcvc-scorekeeper/lib/dtcvc-scoring"

# Configure FastRTPS to force UDP transport in all cases
COPY <<"EOF" /config/fastrtps-profile.xml
<?xml version='1.0' encoding='UTF-8' ?>
<profiles xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles' >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name='participant_profile' is_default_profile='true'>
        <rtps>
            <userTransports>
                <transport_id>CustomUdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
    <publisher profile_name='default publisher profile' is_default_profile='true'>
            <qos>
                <publishMode>
                    <kind>SYNCHRONOUS</kind>
                </publishMode>
            </qos>
            <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        </publisher>
</profiles>
EOF
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastrtps-profile.xml

WORKDIR /

# Add entrypoint script
COPY --chmod=755 <<"EOF" /run.sh
#!/bin/bash

source "/setup.bash"

# Setup for external discovery server
if [[ -n "${DTCVC_DISCOVERY_SERVER_MODE}" && "${DTCVC_DISCOVERY_SERVER_MODE,,}" == "true" ]]; then
    if [[ -n "${ROS_DISCOVERY_SERVER_IP}" && -n "${ROS_DISCOVERY_SERVER_PORT}" ]]; then

        # Export appropriate server location
        if [[ -n "${DTCVC_IS_DISCOVERY_SERVER_HOST}" && "${DTCVC_IS_DISCOVERY_SERVER_HOST,,}" == "true" ]]; then
            export ROS_DISCOVERY_SERVER="localhost:${ROS_DISCOVERY_SERVER_PORT}"
        else
            export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER_IP}:${ROS_DISCOVERY_SERVER_PORT}"
        fi

        # Set configuration for fast RTPS
        if [[ -f "/super_client_configuration_file.template.xml" ]]; then
            if [[ ! -f "/super_client_configuration_file.xml" ]]; then
                cp /super_client_configuration_file.template.xml /super_client_configuration_file.xml

                # NOTE: Current XML parser for discovery server does not resolve environment variables
                # The following `sed` commands manually substitute them
                sed -i "s/\${ROS_DISCOVERY_SERVER_IP}/$(echo ${ROS_DISCOVERY_SERVER_IP} | sed -e 's/[\/&]/\\&/g')/g" "/super_client_configuration_file.xml"
                sed -i "s/\${ROS_DISCOVERY_SERVER_PORT}/$(echo ${ROS_DISCOVERY_SERVER_PORT} | sed -e 's/[\/&]/\\&/g')/g" "/super_client_configuration_file.xml"

                if [[ -n "${ROS_SECURITY_ENABLE}" && "${ROS_SECURITY_ENABLE,,}" != "true" ]]; then
                    # Remove security policy
                    sed -i "/<propertiesPolicy>/,/<\/propertiesPolicy>/d" "/super_client_configuration_file.xml"
                else
                    sed -i "s/\${ROS_SECURITY_KEYSTORE}/$(echo ${ROS_SECURITY_KEYSTORE} | sed -e 's/[\/&]/\\&/g')/g" "/super_client_configuration_file.xml"
                    sed -i "s/\${ROS_SECURITY_ENCLAVE_OVERRIDE}/$(echo ${ROS_SECURITY_ENCLAVE_OVERRIDE} | sed -e 's/[\/&]/\\&/g')/g" "/super_client_configuration_file.xml"
                fi
            fi

            # Necessary for ROS2 daemon to use external discovery server
            export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
            export RMW_FASTRTPS_USE_QOS_FROM_XML=1
            export FASTRTPS_DEFAULT_PROFILES_FILE="/super_client_configuration_file.xml"

            # Refresh daemon
            ros2 daemon stop
            ros2 daemon start
        fi
    fi
fi

exec "$@"
EOF

# https://design.ros2.org/articles/roslaunch.html#termination
# Set stop signal to "Interrupt" instead of "Terminate" 
# to allow ROS2 to gracefully shutdown
STOPSIGNAL SIGINT

ENTRYPOINT [ "/run.sh" ]