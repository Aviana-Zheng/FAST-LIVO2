ARG CUDA_VERSION
ARG CUDNN_VERSION
ARG OS_VERSION

FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-devel-ubuntu${OS_VERSION}

# 这个base没有/usr/share/zoneinfo文件夹，设置是无效的
ENV TZ=Asia/Shanghai

# Set timezone in case of interation during installation
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2004/x86_64/7fa2af80.pub

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y \
        wget \
        gnupg \
        curl \
        vim \
        git \
        unzip \
        python3-dev \
        python3-pip \
        python3-numpy \
        python3-tk \
        libssl-dev \
        libffi-dev \
        zlib1g-dev \
        build-essential \
        cmake \
        iputils-ping \
        openssh-server \
        openssh-client \
        libgoogle-glog-dev \
        libeigen3-dev \
        libsuitesparse-dev \
        libyaml-cpp-dev \
        libtbb-dev \
        libgmock-dev \
        libncurses5-dev \
        libgdbm-dev \
        libnss3-dev \
        libreadline-dev \
        libsqlite3-dev \
        libbz2-dev \
        llvm  \
        xz-utils \
        tk-dev \
        libxml2-dev \
        libxmlsec1-dev  \
        mecab-ipadic-utf8 \
        liblzma-dev \
        libpcl-dev \
        pcl-tools \
        graphviz \
        libopencv-dev \
        libopencv-contrib-dev \
        bear \
        tmux \
        passwd \
        libjsoncpp-dev

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update && apt install ros-noetic-desktop-full -y

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo 'export PATH=$PATH:/usr/local/cuda/bin' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/extras/CUPTI/lib64' >> /root/.bashrc

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y ros-noetic-navigation \
    ros-noetic-robot-localization \
    ros-noetic-robot-state-publisher \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-velodyne-msgs 

RUN pip install rosdepc matplotlib empy catkin_pkg rospkg -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple && \
    pip install --upgrade numpy Flask -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
RUN rosdepc init 
RUN rosdepc update
RUN apt-get update && apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool

ENV PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig:PKG_CONFIG_PATH

WORKDIR /root/
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    cd build && \
    cmake .. && \
    make -j8 install
    # make -j8 install && \
    # rm -rf ../../Livox-SDK

RUN git clone https://gitee.com/aviana-zheng/Sophus && \
    cd Sophus && \
    git checkout a621ff2e_lvalue && \
    mkdir build && cd build && cmake .. && \
    make && \
    make install
    # make install && \
    # rm -rf ../../Sophus

RUN rm -rf /var/lib/apt/lists/*

ENV CPLUS_INCLUDE_PATH=/usr/include/python3.8:$CPLUS_INCLUDE_PATH

# SSH服务相关
RUN mkdir /var/run/sshd && \
    echo 'root:root' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#Port 22/Port 22/' /etc/ssh/sshd_config && \
    sed -i 's/#PubkeyAuthentication yes/PubkeyAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/UsePAM yes/UsePAM no/' /etc/ssh/sshd_config
CMD ["/usr/sbin/sshd", "-D"]

#shell color style setting 
ENV TERM=xterm-256color
ENV force_color_prompt=yes
RUN echo 'export PS1="\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$"' \
            >> /etc/bash.bashrc && \
    echo "export GREP_OPTIONS='--color=auto'" >> /etc/bash.bashrc && \
    echo "export GREP_COLOR='1;35'" >> /etc/bash.bashrc && \
    echo "alias ls='ls --color=auto'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_mb=$'\E[01;31m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_md=$'\E[01;31m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_me=$'\E[0m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_se=$'\E[0m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_so=$'\E[01;44;33m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_ue=$'\E[0m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_us=$'\E[01;32m'" >> /etc/bash.bashrc

SHELL ["/bin/bash", "-c"]

WORKDIR /root/workspace

