FROM ros:noetic

RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

COPY ros_ws /root

VOLUME /root

RUN echo '#!/bin/sh\n\
    set -e\n\
    /usr/sbin/usermod -aG sudo,adm,dialout,video root\n\
    sed -i "s/^%sudo.*/%sudo ALL=(ALL) NOPASSWD: ALL/" /etc/sudoers\n\
    ' > /usr/local/bin/start.sh && \
    chmod +x /usr/local/bin/start.sh && \
    /usr/local/bin/start.sh && \
    rm /usr/local/bin/start.sh

RUN usermod -a -G dialout root
RUN sudo usermod -a -G tty root

CMD ["roslaunch"]


