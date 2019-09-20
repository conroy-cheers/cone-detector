#include <chrono>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header__struct.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "wheeliebot_msgs/msg/detections2_d.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std::chrono_literals;


const int max_value = 255;
int low_H = 190 / 2, low_S = 0.45 * max_value, low_V = 0.25 * max_value;
int high_H = 250 / 2, high_S = max_value, high_V = max_value;


class MinimalPublisher : public rclcpp::Node {
public:
    explicit MinimalPublisher(bool const *is_shutdown) : Node("cone_detector"), count_(0) {
        publisher_ = this->create_publisher<wheeliebot_msgs::msg::Detections2D>("topic", 10);
        is_shutdown_ = is_shutdown;

        cv_img_.encoding = "bgr8";

        cap = cv::VideoCapture(0);
        cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    }

    void run() {
        image_transport::ImageTransport img_transport(shared_from_this());
        img_publisher_ = img_transport.advertise("robot/image", 10);

        cv::SimpleBlobDetector::Params params;
        params.minThreshold = 10;
        params.maxThreshold = 200;

        params.filterByArea = true;
        params.minArea = 300;

        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();

        const int erosion_size = 6;
        const int dilation_size = 5;

        cv::Mat erosion_element = getStructuringElement(cv::MORPH_ELLIPSE,
                                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                        cv::Point(erosion_size, erosion_size));

        cv::Mat dilation_element = getStructuringElement(cv::MORPH_RECT,
                                                         cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                         cv::Point(dilation_size, dilation_size));

        while (!*is_shutdown_) {
            cv::Mat frame, frame_HSV, frame_threshold;
            cap >> frame;

            if (frame.empty()) {
                std::cout << "No image available yet. Waiting...";
                rclcpp::sleep_for(1s);
                continue;
            }

            cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
            cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V),
                        frame_threshold);
            cv::bitwise_not(frame_threshold, frame_threshold);

            cv::dilate(frame_threshold, frame_threshold, dilation_element);
            cv::erode(frame_threshold, frame_threshold, erosion_element);
            cv::erode(frame_threshold, frame_threshold, erosion_element);

            // Detect blobs.
            std::vector<cv::KeyPoint> keypoints;
            cv::cvtColor(frame_threshold, frame_threshold, cv::COLOR_GRAY2BGR);

            detector->detect(frame_threshold, keypoints);


            cv::Mat merged_output;
            cv::min(frame, frame_threshold, merged_output);

            // Draw image with keypoints circled.
            cv::Mat im_with_keypoints;
            cv::drawKeypoints(merged_output, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255),
                              cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            cv_img_.header.stamp = this->now();
            cv_img_.header.frame_id = std::to_string(count_);
            cv_img_.image = im_with_keypoints;

            cv_img_.toImageMsg(image_msg_);
            img_publisher_.publish(image_msg_);

            ++count_;
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
    cv::VideoCapture cap;
    bool const *is_shutdown_;

    sensor_msgs::msg::Image image_msg_;
    cv_bridge::CvImage cv_img_;

    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher img_publisher_;
    rclcpp::Publisher<wheeliebot_msgs::msg::Detections2D>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    bool is_shutdown = false;

    auto publisher = std::make_shared<MinimalPublisher>(&is_shutdown);
    std::thread img_grab_thread([=] { publisher->run(); });

    rclcpp::spin(publisher);

    is_shutdown = true;
    rclcpp::shutdown();

    return 0;
}

