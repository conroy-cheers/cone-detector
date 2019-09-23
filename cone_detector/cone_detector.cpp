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
int low_H = 80 / 2, low_S = 0.3 * max_value, low_V = 0.3 * max_value;
int high_H = 150 / 2, high_S = max_value, high_V = max_value;


class MinimalPublisher : public rclcpp::Node {
public:
    explicit MinimalPublisher(bool const *is_shutdown) : Node("cone_detector"), count_(0) {
        publisher_ = this->create_publisher<wheeliebot_msgs::msg::Detections2D>("topic", 10);
        is_shutdown_ = is_shutdown;

        cv_img_.encoding = "bgr8";

        cap = cv::VideoCapture(0);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    }

    void run() {
        image_transport::ImageTransport img_transport(shared_from_this());
        img_publisher_ = img_transport.advertise("robot/image", 10);

        const int erosion_size = 3;
        const int dilation_size = 2;

        cv::Mat erosion_element = getStructuringElement(cv::MORPH_RECT,
                                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                        cv::Point(erosion_size, erosion_size));

        cv::Mat dilation_element = getStructuringElement(cv::MORPH_RECT,
                                                         cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                         cv::Point(dilation_size, dilation_size));
        long fps;
        long smoothFPS = 0;

        while (!*is_shutdown_) {
            cv::Mat input_frame, frame, frame_HSV, frame_threshold;
            cap >> input_frame;

            if (input_frame.empty()) {
                std::cout << "No image available yet. Waiting...";
                rclcpp::sleep_for(1s);
                continue;
            }

            auto t1 = std::chrono::high_resolution_clock::now();

            cv::resize(input_frame, frame, cv::Size(80, 60), 0, 0, cv::INTER_LINEAR);

            cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
            cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V),
                        frame_threshold);

            cv::dilate(frame_threshold, frame_threshold, dilation_element);
            cv::erode(frame_threshold, frame_threshold, erosion_element);

            // Detect contours
            std::vector<cv::Point> default_largest_contour;

            int largest_area = 0;
            int largest_contour_index = -1;
            std::vector<cv::Rect> bounding_rects;
            std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(frame_threshold, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

            // Find largest contour
            int index = 0;
            for (auto &contour: contours) {
                // Save all bounding rectangles above a given size
                auto rect = cv::boundingRect(contour);
                if (rect.width > 4 && rect.height > 10) {
                    bounding_rects.emplace_back(cv::boundingRect(contour));
                }

                //  Find the area of contour
                auto a = (int) contourArea(contour, false);

                if (a > largest_area) {
                    largest_area = a;
                    // Store the index of largest contour
                    largest_contour_index = index;
                }
                ++index;
            }

            cv::Mat merged_output;
            cv::cvtColor(frame_threshold, frame_threshold, cv::COLOR_GRAY2BGR);
            cv::max(frame, frame_threshold, merged_output);

            if (largest_contour_index > -1) {
                cv::drawContours(merged_output, contours, largest_contour_index, cv::Scalar(0, 0, 255), CV_FILLED, 8,
                                 hierarchy);
            }
            for (auto &rect: bounding_rects) {
                cv::rectangle(merged_output, rect, cv::Scalar(0, 255, 0), 2);
            }

            auto t2 = std::chrono::high_resolution_clock::now();
            fps = 1000000000 / std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
            smoothFPS = (9 * smoothFPS + fps) / 10;

            std::cout << smoothFPS << " FPS" << std::endl;

            cv_img_.header.stamp = this->now();
            cv_img_.header.frame_id = std::to_string(count_);
            cv_img_.image = merged_output;

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

