// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mynteye_wrapper_d/srv/get_params.hpp>
#include <sensor_msgs/msg/image.hpp>

#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;  // NOLINT

namespace enc = sensor_msgs::image_encodings;

using namespace std::chrono_literals;

class MYNTEYEListener : public rclcpp::Node {
 public:
    MYNTEYEListener()
        : Node("mynteye_listener"),
          color_count_(0),
          it_(this->shared_from_this()) {
        color_sub_ = it_.subscribe("mynteye/left/image_color", 1,
                                   &MYNTEYEListener::colorCallback, this);
        depth_sub_ = it_.subscribe("mynteye/depth/image_raw", 1,
                                   &MYNTEYEListener::depthCallback, this);

        std::string img_intri = getCameraCalibInfo(0u);
        std::string img_extri = getCameraCalibInfo(1u);
        std::string imu_intri = getCameraCalibInfo(2u);
        std::string imu_extri = getCameraCalibInfo(3u);

        auto img_intri_info = parse_string(img_intri.c_str(), JSON, "log");
        auto img_extri_info = parse_string(img_extri.c_str(), JSON, "log");
        auto imu_intri_info = parse_string(imu_intri.c_str(), JSON, "log");
        auto imu_extri_info = parse_string(imu_extri.c_str(), JSON, "log");
        std::cout << "IMG_INTRINSICS:" << img_intri_info << std::endl
                  << "IMG_EXTRINSICS_RTOL:" << img_extri_info << std::endl
                  << "IMU_INTRINSICS:" << imu_intri_info << std::endl
                  << "IMU_EXTRINSICS:" << imu_extri_info << std::endl;

        cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
    }

    ~MYNTEYEListener() { cv::destroyAllWindows(); }

    std::string getCameraCalibInfo(unsigned int type_) {
        auto client = this->create_client<mynteye_wrapper_d::srv::GetParams>(
            "/mynteye_wrapper_d_node/get_params");
        auto request =
            std::make_shared<mynteye_wrapper_d::srv::GetParams::Request>();
        request->key = type_;

        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for service");
                return "null";
            }
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            return result.get()->value;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to call service GetParams, make sure you have "
                         "launch mynteye device SDK nodelet");
            return "null";
        }
    }

    void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "cv_bridge exception: " << e.what());
            return;
        }
        ++color_count_;
        // ROS_INFO_STREAM("color: " << color_count_);

        cv::imshow("color", cv_ptr->image);
        cv::waitKey(3);
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            if (enc::isColor(msg->encoding)) {
                cv_ptr = cv_bridge::toCvShare(msg, enc::RGB8);
            } else if (msg->encoding == enc::MONO16) {
                cv_ptr = cv_bridge::toCvShare(msg, enc::MONO16);
            } else {
                cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "cv_bridge exception: " << e.what());
            return;
        }
        cv::imshow("depth", cv_ptr->image);
        cv::waitKey(3);
    }

    std::uint64_t colorCount() const { return color_count_; }

 private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber color_sub_;
    image_transport::Subscriber depth_sub_;

    std::uint64_t color_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto listener = std::make_shared<MYNTEYEListener>();

    double time_beg = rclcpp::Clock().now().seconds();
    rclcpp::spin(listener);
    double time_end = rclcpp::Clock().now().seconds();

    double elapsed = time_end - time_beg;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("mynteye_listener"),
                       "time beg: " << std::fixed << time_beg << " s");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("mynteye_listener"),
                       "time end: " << std::fixed << time_end << " s");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("mynteye_listener"),
                       "time cost: " << elapsed << " s");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("mynteye_listener"),
                       "color count: " << listener->colorCount() << ", "
                                       << (listener->colorCount() / elapsed)
                                       << " fps");

    return 0;
}
