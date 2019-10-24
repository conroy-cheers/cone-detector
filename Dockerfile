FROM conroycheers/ros2-gstreamer:latest

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
	python3-pip \
	pkg-config \
	libcairo2-dev \
	gcc \
	git \
	python3-dev \
	python3-smbus \
	libopencv-dev \
	libboost-dev \
	python3-opencv

RUN pip3 install pyserial
RUN apt-get install -y libboost-python-dev

RUN cd src && git clone https://github.com/ros-perception/vision_opencv.git --branch dashing
RUN rm -rf src/vision_opencv/image_geometry src/vision_opencv/opencv_tests src/vision_opencv/vision_opencv
RUN cd src && git clone https://github.com/ros-perception/image_common.git --branch ros2 && git clone https://github.com/ros2/yaml_cpp_vendor.git
RUN source install/setup.bash && colcon build --packages-skip cone_detector wheelie_serial wheeliebot_msgs wheeliebot

ADD . src/
RUN source install/setup.bash && colcon build --packages-select cone_detector wheelie_serial wheeliebot_msgs wheeliebot

ENV ROS_DOMAIN_ID=42
ADD wheeliebot_env.sh ./

CMD ["/bin/bash", "-c", "source install/setup.bash && source wheeliebot_env.sh && ros2 launch wheeliebot wheeliebot.launch.py"]

