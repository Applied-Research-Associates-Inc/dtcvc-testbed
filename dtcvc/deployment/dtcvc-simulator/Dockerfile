FROM ubuntu:jammy


## Install dependencies
RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y \
        xdg-user-dirs \
        xdg-utils \
        libxext6 \
        xorg-dev \
        pciutils \
        vulkan-tools \
        mesa-utils \
        libatomic1 \
        net-tools \
    && rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Configure Nvidia Vulkan execution inside container
RUN /bin/bash <<-EOF
    set -e
    VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'`
    mkdir -p /etc/vulkan/icd.d/
    cat <<- EOF2 > /etc/vulkan/icd.d/nvidia_icd.json
        {
            "file_format_version" : "1.0.0",
            "ICD": {
                "library_path": "libGLX_nvidia.so.0",
                "api_version" : "${VULKAN_API_VERSION}"
            }
        }
    EOF2
EOF

# Add custom entrypoint
COPY --chmod=755 <<EOF /run.sh
#! /bin/bash

/home/ueuser/dtcvc-simulator/CarlaUnreal.sh --ros2 -RenderOffScreen \$@
EOF

# Add UE5 User
RUN <<EOF
groupadd -r ueuser
useradd -r -u 1000 -g ueuser -G audio,video -m -d /home/ueuser -s /bin/bash ueuser
chmod 755 /home/ueuser
EOF

# Set user and provide permissions 
USER ueuser
WORKDIR /home/ueuser

# Copy simulator into user home
COPY --from=external-packages --chown=ueuser:ueuser ./Linux /home/ueuser/dtcvc-simulator

ENTRYPOINT [ "/run.sh" ]
