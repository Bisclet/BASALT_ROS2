/**
BSD 3-Clause License

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <thread>

#include <fmt/format.h>

#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/global_control.h>

#include <CLI/CLI.hpp>

#include <basalt/io/dataset_io.h>
#include <basalt/io/marg_data_io.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/calibration/calibration.hpp>

#include <basalt/serialization/headers_serialization.h>

#include <basalt/utils/system_utils.h>
#include <basalt/utils/vis_utils.h>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>

#include "basalt/utils/eigen_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>



// enable the "..."_format(...) string literal
using namespace basalt::literals;


// Visualization variables




// VIO variables


// Feed functions
/*
void feed_images() {
  std::cout << "Started input_data thread " << std::endl;

  for (size_t i = 0; i < vio_dataset->get_image_timestamps().size(); i++) {
    if (vio->finished || terminate || (max_frames > 0 && i >= max_frames)) {
      // stop loop early if we set a limit on number of frames to process
      break;
    }

    if (step_by_step) {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk);
    }

    basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

    data->t_ns = vio_dataset->get_image_timestamps()[i];
    data->img_data = vio_dataset->get_image_data(data->t_ns);

    timestamp_to_id[data->t_ns] = i;

    opt_flow_ptr->input_queue.push(data);
  }

  // Indicate the end of the sequence
  opt_flow_ptr->input_queue.push(nullptr);

  std::cout << "Finished input_data thread " << std::endl;
}

void feed_imu() {
  for (size_t i = 0; i < vio_dataset->get_gyro_data().size(); i++) {
    if (vio->finished || terminate) {
      break;
    }

    basalt::ImuData<double>::Ptr data(new basalt::ImuData<double>);
    data->t_ns = vio_dataset->get_gyro_data()[i].timestamp_ns;

    data->accel = vio_dataset->get_accel_data()[i].data;
    data->gyro = vio_dataset->get_gyro_data()[i].data;

    vio->imu_data_queue.push(data);
  }
  vio->imu_data_queue.push(nullptr);
}
*/

using std::placeholders::_1;
using std::placeholders::_2;
using namespace message_filters;
using Image    = sensor_msgs::msg::Image;
using Imu      = sensor_msgs::msg::Imu;
using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;
using ImuPtr   = sensor_msgs::msg::Imu::ConstSharedPtr;

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;


class BasaltVIO : public rclcpp::Node{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        Subscriber<Image> left_sub_;
        Subscriber<Image> right_sub_;
        std::shared_ptr<Synchronizer<SyncPolicy>> sync_;

        basalt::Calibration<double> calib;

        basalt::VioConfig vio_config;
        basalt::OpticalFlowBase::Ptr opt_flow_ptr;
        basalt::VioEstimatorBase::Ptr vio;

        

        std::string left_topic_, right_topic_, imu_topic_, odom_topic_, cam_calib, config_path;
        std::atomic<bool> terminate = false;

        int counter;

        tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>
            out_state_queue;

        std::vector<int64_t> vio_t_ns;
        Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;
        Eigen::aligned_vector<Sophus::SE3d> vio_T_w_i;

        std::vector<int64_t> gt_t_ns;
        Eigen::aligned_vector<Eigen::Vector3d> gt_t_w_i;

        std::string marg_data_path;

        tbb::concurrent_unordered_map<int64_t, int, std::hash<int64_t>> timestamp_to_id;

        std::mutex m;
        std::condition_variable cv;

        void load_data(const std::string& calib_path) {
            std::ifstream os(calib_path, std::ios::binary);

            if (os.is_open()) {
                cereal::JSONInputArchive archive(os);
                archive(calib);
                std::cout << "Loaded camera with " << calib.intrinsics.size() << " cameras"
                        << std::endl;

            } else {
                std::cerr << "could not load camera calibration " << calib_path
                        << std::endl;
                std::abort();
            }
        }

        void imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            if (vio->finished || terminate) {
                RCLCPP_INFO(get_logger(), "IMU ignored, VIO finished!");
                vio->imu_data_queue.push(nullptr);
                return;
            }
            
            auto stamp = msg->header.stamp;
            int64_t t_ns =
                static_cast<int64_t>(stamp.sec) * 1000000000LL +
                static_cast<int64_t>(stamp.nanosec);

            auto data = basalt::ImuData<double>::Ptr(new basalt::ImuData<double>);
            data->t_ns = t_ns;
            
            data->accel << msg->linear_acceleration.x, 
                        msg->linear_acceleration.y, 
                        msg->linear_acceleration.z;
            
            data->gyro << msg->angular_velocity.x, 
                        msg->angular_velocity.y, 
                        msg->angular_velocity.z;
            
            vio->imu_data_queue.push(data);
        }

        void stereo_callback(const ImagePtr& left, const ImagePtr& right) {
            if (vio->finished || terminate) {
                RCLCPP_INFO(get_logger(), "Images ignored, VIO finished!");
                opt_flow_ptr->input_queue.push(nullptr);
                return;
            }
            
            auto stamp = left->header.stamp;
            int64_t t_ns =
                static_cast<int64_t>(stamp.sec) * 1000000000LL +
                static_cast<int64_t>(stamp.nanosec);

            auto data = basalt::OpticalFlowInput::Ptr(new basalt::OpticalFlowInput);
            data->t_ns = t_ns;
            
            // Convert ROS images to basalt ImageData format
            data->img_data.resize(2); // For stereo: left (0) and right (1)
            
            // Convert left image
            cv_bridge::CvImageConstPtr cv_left = cv_bridge::toCvShare(left, sensor_msgs::image_encodings::MONO8);
            data->img_data[0].img.reset(new basalt::ManagedImage<uint16_t>(cv_left->image.cols, cv_left->image.rows));
            
            // Copy and convert from uint8 to uint16
            for (int y = 0; y < cv_left->image.rows; y++) {
                for (int x = 0; x < cv_left->image.cols; x++) {
                    (*data->img_data[0].img)(x, y) = cv_left->image.at<uint8_t>(y, x) << 8;
                }
            }
            
            // Convert right image
            cv_bridge::CvImageConstPtr cv_right = cv_bridge::toCvShare(right, sensor_msgs::image_encodings::MONO8);
            data->img_data[1].img.reset(new basalt::ManagedImage<uint16_t>(cv_right->image.cols, cv_right->image.rows));
            
            // Copy and convert from uint8 to uint16
            for (int y = 0; y < cv_right->image.rows; y++) {
                for (int x = 0; x < cv_right->image.cols; x++) {
                    (*data->img_data[1].img)(x, y) = cv_right->image.at<uint8_t>(y, x) << 8;
                }
            }
            
            timestamp_to_id[data->t_ns] = counter++;
            opt_flow_ptr->input_queue.push(data);
        }

        void setup() {

            if (!config_path.empty()) {
                vio_config.load(config_path);
            }

            load_data(cam_calib);

            opt_flow_ptr = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
        
            vio = basalt::VioEstimatorFactory::getVioEstimator(
                vio_config, calib, basalt::constants::g, true, true);
            vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

            opt_flow_ptr->output_queue = &vio->vision_data_queue;
            vio->out_state_queue = &out_state_queue;
            basalt::MargDataSaver::Ptr marg_data_saver;

            if (!marg_data_path.empty()) {
                marg_data_saver.reset(new basalt::MargDataSaver(marg_data_path));
                vio->out_marg_queue = &marg_data_saver->in_marg_queue;

                // Save gt.
                {
                std::string p = marg_data_path + "/gt.cereal";
                std::ofstream os(p, std::ios::binary);

                {
                    cereal::BinaryOutputArchive archive(os);
                    archive(gt_t_ns);
                    archive(gt_t_w_i);
                }
                os.close();
                }
            }
            RCLCPP_INFO(get_logger(), "Finished setup");
        }

    public:
        BasaltVIO() : Node("BasaltVIO") {
            declare_parameter<std::string>("left_image_topic",  "/cam0/image_raw");
            declare_parameter<std::string>("right_image_topic", "/cam1/image_raw");
            declare_parameter<std::string>("imu_topic",         "/imu0");
            declare_parameter<std::string>("odometry_topic",    "/orb_slam/odom");
            declare_parameter<std::string>("cam_calib",      "file_not_set");
            declare_parameter<std::string>("config_path", "file_not_set");
            

            left_topic_     = get_parameter("left_image_topic").as_string();
            right_topic_    = get_parameter("right_image_topic").as_string();
            imu_topic_      = get_parameter("imu_topic").as_string();
            odom_topic_     = get_parameter("odometry_topic").as_string();
            cam_calib       = get_parameter("cam_calib").as_string();
            config_path  = get_parameter("config_path").as_string();


            imu_sub_ = this->create_subscription<Imu>(
                imu_topic_,
                rclcpp::SensorDataQoS().keep_last(400),
                std::bind(&BasaltVIO::imu_callback, this, _1)
            );

            rclcpp::QoS qos = rclcpp::SensorDataQoS();

            left_sub_.subscribe(this, left_topic_, qos.get_rmw_qos_profile());
            right_sub_.subscribe(this, right_topic_, qos.get_rmw_qos_profile());

            sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(10), left_sub_, right_sub_);
            sync_->registerCallback(std::bind(&BasaltVIO::stereo_callback, this, _1, _2));

            rclcpp::QoS qosi(rclcpp::KeepLast(10));
            qosi.reliable();
            qosi.durability_volatile();

            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                odom_topic_, qosi);

        
            setup();


            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20),
                [this]() {
                    basalt::PoseVelBiasState<double>::Ptr data;
                    out_state_queue.try_pop(data);

                    if (!data.get()) { return;}

                    // std::cerr << "t_ns " << t_ns << std::endl;
                    Sophus::SE3d T_w_i = data->T_w_i;
                    Eigen::Vector3d vel_w_i = data->vel_w_i;
                    //Eigen::Vector3d bg = data->bias_gyro;
                    //Eigen::Vector3d ba = data->bias_accel;

                    nav_msgs::msg::Odometry odom_msg;

                    odom_msg.header.stamp = rclcpp::Time(data->t_ns);
                    odom_msg.header.frame_id = "map";  // or "odom", "map", etc.
                    odom_msg.child_frame_id = "base_link";  // or "imu", "camera", etc.

                    odom_msg.pose.pose.position.x = T_w_i.translation().x();
                    odom_msg.pose.pose.position.y = T_w_i.translation().y();
                    odom_msg.pose.pose.position.z = T_w_i.translation().z();

                    Eigen::Quaterniond quat = T_w_i.unit_quaternion();
                    odom_msg.pose.pose.orientation.x = quat.x();
                    odom_msg.pose.pose.orientation.y = quat.y();
                    odom_msg.pose.pose.orientation.z = quat.z();
                    odom_msg.pose.pose.orientation.w = quat.w();

                    odom_msg.twist.twist.linear.x = vel_w_i[0];
                    odom_msg.twist.twist.linear.y = vel_w_i[1];
                    odom_msg.twist.twist.linear.z = vel_w_i[2];

                    odom_pub_->publish(odom_msg);

                    //vio_t_ns.emplace_back(data->t_ns);
                    //vio_t_w_i.emplace_back(T_w_i.translation());
                    //vio_T_w_i.emplace_back(T_w_i);
                }
            );

            RCLCPP_INFO(get_logger(), "Node starting...");
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<BasaltVIO>();

        RCLCPP_INFO(node->get_logger(), "BasaltVIO node started");
        RCLCPP_INFO(node->get_logger(), "Waiting for sensor data...");

        // Configure executor with number of threads
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(),
            4  // Number of threads - adjust based on your needs
        );
        
        executor.add_node(node);
        executor.spin();

        RCLCPP_INFO(node->get_logger(), "BasaltVIO node shutting down");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("BasaltVIO"),
            "Exception in main: %s", e.what()
        );
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
