FROM althack/ros2:galactic-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends ros-galactic-cv-bridge ros-galactic-ros2-socketcan \
  ros-galactic-robot-localization ros-galactic-lgsvl-msgs libboost-all-dev libasio-dev \
  python3-catkin-pkg ros-galactic-ackermann-msgs libboost-dev ros-galactic-udp-msgs \
  ros-galactic-example-interfaces \
  #
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN pip install opencv-python
RUN pip install scipy

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

#RUN . /opt/ros/galactic/setup.bash
#RUN rosdep update
#RUN pwd
#RUN ls
#RUN rosdep install --from-paths /workspaces/triton-ai-ros2/src --ignore-src -y -r