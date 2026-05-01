FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV OPENVINO_VERSION=2024.6.0
ENV OPENVINO_ARCHIVE=l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz
ENV OPENVINO_URL=https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/${OPENVINO_ARCHIVE}
ENV OpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
ENV LD_LIBRARY_PATH=/opt/intel/openvino_2024.6.0/runtime/lib/intel64:${LD_LIBRARY_PATH}

RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g; s|http://security.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      git \
      libceres-dev \
      libeigen3-dev \
      libfmt-dev \
      libopencv-dev \
      libpaho-mqtt-dev \
      libpaho-mqttpp-dev \
      libssl-dev \
      libspdlog-dev \
      libusb-1.0-0-dev \
      libyaml-cpp-dev \
      mosquitto-clients \
      nlohmann-json3-dev \
      pkg-config \
      python3 \
      python3-pip \
      tar \
      unzip && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/intel && \
    curl -fsSL "${OPENVINO_URL}" -o /tmp/openvino.tgz && \
    tar -xzf /tmp/openvino.tgz -C /opt/intel && \
    rm /tmp/openvino.tgz && \
    ln -sfn /opt/intel/l_openvino_toolkit_ubuntu22_${OPENVINO_VERSION}.17404.4c0f47d2335_x86_64 /opt/intel/openvino_2024.6.0 && \
    echo /opt/intel/openvino_2024.6.0/runtime/lib/intel64 > /etc/ld.so.conf.d/openvino.conf && \
    ldconfig

WORKDIR /app

CMD ["/bin/bash"]
