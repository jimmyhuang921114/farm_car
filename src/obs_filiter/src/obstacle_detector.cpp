#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <map>

class ObstacleNode : public rclcpp::Node {
public:
    ObstacleNode() : Node("obstacle_node") {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/plane_cloud", 10, std::bind(&ObstacleNode::cloud_callback, this, std::placeholders::_1));

        coeff_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/plane_coeff", 10, std::bind(&ObstacleNode::coeff_callback, this, std::placeholders::_1));

        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", 10, std::bind(&ObstacleNode::color_callback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_cloud", 10);
        proj_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_xz", 10);
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/obstacle/image_raw", 10);
        nearest_info_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/obstacle/nearest", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr coeff_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr proj_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr nearest_info_pub_;

    std::vector<float> latest_coeff_;
    cv::Mat latest_color_;
    std::mutex color_mutex_;

    void coeff_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
            latest_coeff_ = msg->data;
        }
    }

    void color_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(color_mutex_);
        try {
            latest_color_ = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (latest_coeff_.size() < 4) return;

        cv::Mat vis;
        {
            std::lock_guard<std::mutex> lock(color_mutex_);
            if (latest_color_.empty()) return;
            vis = latest_color_.clone();
        }

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);

        const float a = latest_coeff_[0];
        const float b = latest_coeff_[1];
        const float c = latest_coeff_[2];
        const float d = latest_coeff_[3];

        pcl::PointCloud<pcl::PointXYZ>::Ptr xz_proj(new pcl::PointCloud<pcl::PointXYZ>());
        std::map<int, float> angle_to_min_z;

        for (const auto& pt : cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (pt.z < 0.2f || pt.z > 5.0f) continue;

            float dist = a * pt.x + b * pt.y + c * pt.z + d;
            if (!std::isfinite(dist)) continue;

            float fx = 925, fy = 925, cx = 640, cy = 360;
            int u = static_cast<int>(pt.x * fx / pt.z + cx);
            int v = static_cast<int>(pt.y * fy / pt.z + cy);

            if (u >= 0 && u < vis.cols && v >= 0 && v < vis.rows) {
                if (dist > 0.1f) {
                    vis.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 255);
                    xz_proj->emplace_back(pt.x, 0.0f, pt.z);

                    float angle_rad = std::atan2(pt.x, pt.z);
                    int angle_deg = static_cast<int>(std::round(angle_rad * 180.0f / M_PI));
                    if (angle_to_min_z.find(angle_deg) == angle_to_min_z.end() || pt.z < angle_to_min_z[angle_deg]) {
                        angle_to_min_z[angle_deg] = pt.z;
                    }
                } else if (dist > 0.02f && dist <= 0.05f) {
                    vis.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 255, 255);
                } else if (dist <= 0.02f) {
                    vis.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 255, 0);
                } else {
                    vis.at<cv::Vec3b>(v, u) = cv::Vec3b(50, 50, 50);
                }
            }
        }

        auto msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vis).toImageMsg();
        msg_img->header = msg->header;
        color_pub_->publish(*msg_img);

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(cloud, out_msg);
        out_msg.header = msg->header;
        cloud_pub_->publish(out_msg);

        sensor_msgs::msg::PointCloud2 proj_msg;
        pcl::toROSMsg(*xz_proj, proj_msg);
        proj_msg.header = msg->header;
        proj_pub_->publish(proj_msg);

        std_msgs::msg::Float32MultiArray msg_info;
        for (const auto& [angle, min_z] : angle_to_min_z) {
            msg_info.data.push_back(static_cast<float>(angle));
            msg_info.data.push_back(min_z);
        }
        if (!msg_info.data.empty()) {
            nearest_info_pub_->publish(msg_info);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleNode>());
    rclcpp::shutdown();
    return 0;
}
