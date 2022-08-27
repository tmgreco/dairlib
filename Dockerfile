FROM ros:noetic-ros-base-focal
WORKDIR /dairlib
ENV DAIRLIB_DOCKER_VERSION=0.21
COPY . .

RUN DEBIAN_FRONTEND='noninteractive' \ 
    apt-get update && apt-get install -y --no-install-recommends apt-transport-https curl ca-certificates gnupg lsb-release wget
RUN wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
  | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
RUN echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg \ 
  && sudo mv bazel-archive-keyring.gpg /usr/share/keyrings \
  && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list

RUN DEBIAN_FRONTEND='noninteractive' \ 
     apt-get update -y && apt-get install -y wget lsb-release pkg-config zip g++ zlib1g-dev unzip python git neovim nano
RUN DEBIAN_FRONTEND='noninteractive' \
     apt-get install -y  --no-install-recommends xorg xserver-xorg-input-evdev  xserver-xorg-input-all
RUN set -eux \
  && apt-get install --no-install-recommends  locales \
  && locale-gen en_US.UTF-8
RUN if type sudo 2>/dev/null; then \ 
     echo "The sudo command already exists... Skipping."; \
    else \
     echo -e "#!/bin/sh\n\${@}" > /usr/sbin/sudo; \
     chmod +x /usr/sbin/sudo; \
    fi
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && rm -rf /var/lib/apt/lists/*
WORKDIR /
RUN git clone https://github.com/KodlabPenn/drake.git
RUN cd drake \ 
 && git checkout zf-branch \
 && sudo yes | ./setup/ubuntu/install_prereqs.sh
ENV DAIRLIB_LOCAL_DRAKE_PATH="/drake"
ENV DAIRLIB_PATH="/dairlib/"
# RUN apt update && apt-get --only-upgrade install bazel
RUN apt-get install -y lcm libbot2
RUN cd /dairlib && sed -i 's/WITH_SNOPT=ON/WITH_SNOPT=OFF/g' .bazelrc && bazel build
