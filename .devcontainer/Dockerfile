FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
  && apt-get -y install --no-install-recommends ros-humble-cv-bridge ros-humble-ros2-socketcan \
  ros-humble-robot-localization ros-humble-lgsvl-msgs libboost-all-dev libasio-dev \
  python3-dev \
  python3-catkin-pkg ros-humble-ackermann-msgs libboost-dev ros-humble-udp-msgs \
  ros-humble-example-interfaces \
  ros-humble-joy \ 
  #
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN pip install --upgrade numpy>=1.19.0
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
RUN pip install docopt
RUN pip install tornado
RUN pip install requests
RUN pip install h5py
RUN pip install PrettyTable
RUN pip install paho-mqtt
RUN pip install simple_pid
RUN pip install progress
RUN pip install typing_extensions
RUN pip install pyfiglet
RUN pip install psutil
RUN pip install pynmea2
RUN pip install pyserial
RUN pip install utm
RUN pip install setuptools
RUN pip install pytorch-lightning
RUN pip install tensorboard
RUN pip install tensorboardx

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

ENV PYTHONPATH "${PYTHONPATH}:/home/ros/.local/lib/python3.8/site-packages"
ENV HOST_ADDR="192.168.0.219"
ENV DATA_DIR = 'image_and_control'
ENV DATA_FILE = 'image_and_control_0.db3'

#ENV CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><General><ExternalNetworkAddress>${HOST_ADDR}</ExternalNetworkAddress><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>1</ParticipantIndex><Peers><Peer address='${HOST_ADDR}'/></Peers></Discovery><Tracing><Verbosity>config</Verbosity><Out>stderr</Out></Tracing></Domain></CycloneDDS>"