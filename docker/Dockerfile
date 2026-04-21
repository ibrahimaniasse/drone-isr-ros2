FROM osrf/ros:humble-desktop

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-ros-gz \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install specific Python libraries needed for the project
RUN pip3 install --no-cache-dir \
    pytest \
    numpy \
    pyyaml \
    ultralytics \
    opencv-python-headless

# Setup Workspace
ENV WORKSPACE=/isr_ws
WORKDIR $WORKSPACE

# Copy the source code
COPY ./src $WORKSPACE/src



# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add entrypoint sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
