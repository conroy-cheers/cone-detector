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

ENV WHEELIE_CONE_X_BASE=0.6
ENV WHEELIE_CONE_X_PER_AREA=0.4
ENV WHEELIE_CONE_MIN_HEIGHT=0.1
ENV WHEELIE_TRACKING_X_SPEED=0.08
ENV WHEELIE_TURN_BEGIN_AREA_THRESHOLD=0.25
ENV WHEELIE_TURN_X_SPEED=0.03
ENV WHEELIE_TURN_STEER_SPEED=0.05
ENV WHEELIE_TRACKING_K_P=0.05

CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 launch wheeliebot wheeliebot.launch.py"]

