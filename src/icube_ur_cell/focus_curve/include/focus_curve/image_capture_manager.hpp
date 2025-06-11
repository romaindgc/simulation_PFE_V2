#pragma once

#include <mutex>
#include <future>
#include <optional>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

class ImageCaptureManager {
public:
    ImageCaptureManager(rclcpp::Node::SharedPtr node, const std::string& topic_name)
        : node_(node) {
        subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            topic_name, 10,
            std::bind(&ImageCaptureManager::image_callback, this, std::placeholders::_1));
    }

    /// Capture la prochaine image reçue
    cv::Mat get_next_image(std::chrono::seconds timeout = std::chrono::seconds(5)) {
        std::promise<cv::Mat> prom;
        auto fut = prom.get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            image_request_ = std::move(prom);
        }

        if (fut.wait_for(timeout) == std::future_status::ready) {
            return fut.get();
        } else {
            throw std::runtime_error("Timeout while waiting for image.");
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto cv_image = cv_bridge::toCvCopy(msg, msg->encoding);
            std::lock_guard<std::mutex> lock(mutex_);
            if (image_request_) {
                image_request_->set_value(cv_image->image.clone());
                image_request_.reset();  // On efface la promesse une fois remplie
            }
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::mutex mutex_;
    std::optional<std::promise<cv::Mat>> image_request_;
};
