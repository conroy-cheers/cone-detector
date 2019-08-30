#include <chrono>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header__struct.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node {
public:
    explicit MinimalPublisher(bool const *is_shutdown) : Node("cone_detector"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
                500ms, std::bind(&MinimalPublisher::timer_callback, this));
        is_shutdown_ = is_shutdown;

        cv_img_.header.frame_id = "body";
        cv_img_.encoding = "bgr8";

        cap = cv::VideoCapture(0);
    }

    void run() {
        image_transport::ImageTransport img_transport(shared_from_this());
        img_publisher_ = img_transport.advertise("image", 10);

        while (!*is_shutdown_) {
            cv::Mat frame;
            cap >> frame;

            // If the frame is empty, break immediately
//            if (frame.empty())
//                break;

            cv_img_.header.stamp = this->now();
            cv_img_.image = frame;

            cv_img_.toImageMsg(image_msg_);
            img_publisher_.publish(image_msg_);
        }

        shutdown();
    }

    void shutdown() {
        // When everything done, release the video capture object
        cap.release();

        // Closes all the frames
        cv::destroyAllWindows();
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    cv::VideoCapture cap;
    bool const *is_shutdown_;

    sensor_msgs::msg::Image image_msg_;
    cv_bridge::CvImage cv_img_;

    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher img_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    bool is_shutdown = false;

    auto publisher = std::make_shared<MinimalPublisher>(&is_shutdown);
    std::thread img_grab_thread( [=] { publisher->run(); } );

    rclcpp::spin(publisher);

    is_shutdown = true;
    rclcpp::shutdown();

    return 0;
}

