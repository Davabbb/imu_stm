FROM ros:humble-ros-base@sha256:d254bb722b9b490d3b71723dc2a55591170daad2851ad45e367114c211889120

RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y libboost-all-dev
#RUN apt-get update && apt-get install -y \
#    ros-melodic-rosidl-generator-cpp
RUN apt-get install -y ros-humble-sensor-msgs

WORKDIR /root

COPY ros_ws /root

VOLUME /root

CMD ["bash"]


