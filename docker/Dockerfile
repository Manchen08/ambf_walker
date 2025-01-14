
    
FROM ros:kinetic

RUN apt update && apt upgrade -y
# Install necessary nvidia components (comment if not needed)
RUN apt-get update && apt-get install -y --no-install-recommends \
 gnupg2 curl ca-certificates && \
 curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub | apt-key add - && \
 echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
 echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2004/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
 apt-get purge --autoremove -y curl 
ENV CUDA_VERSION 11.1.1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
 cuda-cudart-11-1=11.1.74-1 \
 cuda-compat-11-1 \
 && ln -s cuda-11.1 /usr/local/cuda
# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
 echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${​​​​​​​​PATH}​​​​​​​​
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.1 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 brand=tesla,driver>=450,driver<451"
# Install necessary components
RUN apt update && apt install --no-install-recommends -y \
 ros-noetic-gazebo-ros \
 ros-noetic-gazebo-ros-control \
 ros-noetic-robot-state-publisher \
 ros-noetic-xacro \
 ros-noetic-ros-control \
 ros-noetic-ros-controllers \
 ros-noetic-controller-manager
RUN rm -rf /var/lib/apt/lists/*
# Set up Catkin Workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "CATKIN_WS=${​​​​​​​​CATKIN_WS}​​​​​​​​" >> ~/.bashrc
WORKDIR $CATKIN_WS

