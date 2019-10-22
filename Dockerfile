FROM conroycheers/ros2-gstreamer:latest

SHELL ["/bin/bash", "-c"]
RUN apt-get updatte && apt-get install -y curl gnupg2 lsb-release \
	python3-pip
	pkg-config \
	libcairo2-dev \
	gcc \
	python3-dev \
	python3-smbus

RUN pip3 install pyserial

ADD . /ros2_ws/src
RUN source install/setup.bash && colcon build

ENV ROS_DOMAIN_ID=42
ENTRYPOINT /bin/bash

