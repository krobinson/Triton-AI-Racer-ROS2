FROM althack/ros2:foxy-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends ros-foxy-cv-bridge ros-foxy-ros2-socketcan \
  ros-foxy-robot-localization ros-foxy-lgsvl-msgs libboost-all-dev libasio-dev \
  python3-catkin-pkg ros-foxy-ackermann-msgs libboost-dev ros-foxy-udp-msgs \
  ros-foxy-example-interfaces \
  ros-foxy-joy \ 
  #
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN pip install --upgrade numpy>=1.18.0
RUN pip install opencv-python
RUN pip install pillow
RUN pip install scipy
RUN pip install fastai
RUN pip install torch
RUN pip install torchvision
RUN pip install git+https://github.com/tawnkramer/gym-donkeycar
# Donkeycar python
RUN pip install progress
RUN pip install prettytable

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

ENV PYTHONPATH "${PYTHONPATH}:/home/ros/.local/lib/python3.8/site-packages"
ENV HOST_ADDR="192.168.0.219"
ENV DATA_DIR = 'image_and_control'
ENV DATA_FILE = 'image_and_control_0.db3'

#ENV CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><General><ExternalNetworkAddress>${HOST_ADDR}</ExternalNetworkAddress><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>1</ParticipantIndex><Peers><Peer address='${HOST_ADDR}'/></Peers></Discovery><Tracing><Verbosity>config</Verbosity><Out>stderr</Out></Tracing></Domain></CycloneDDS>"