#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <map>

class XZVisualizerNode : public rclcpp::Node {
public:
    XZVisualizerNode() : Node("xz_visualizer_node") {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacle_xz", 10, std::bind(&XZVisualizerNode::cloud_callback, this, std::placeholders::_1));
        nearest_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/obstacle/nearest", 10, std::bind(&XZVisualizerNode::nearest_callback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/obstacle_xz_image", 10);

        cv::namedWindow("Obstacle Grid View", cv::WINDOW_AUTOSIZE);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr nearest_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    std::map<int, float> angle_distance_map_;

    void nearest_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        angle_distance_map_.clear();
        for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
            int angle = static_cast<int>(std::round(msg->data[i]));
            float dist = msg->data[i + 1];
            angle_distance_map_[angle] = dist;
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        const float grid_resolution = 0.1f;
        const float x_min = -3.0f, x_max = 3.0f;
        const float z_min = 0.0f, z_max = 5.0f;
        const int cols = static_cast<int>((x_max - x_min) / grid_resolution);
        const int rows = static_cast<int>((z_max - z_min) / grid_resolution);
        const int scale = 10;

        std::vector<std::vector<bool>> occupancy(rows, std::vector<bool>(cols, false));

        for (const auto& pt : cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.z)) continue;
            int col = static_cast<int>((pt.x - x_min) / grid_resolution);
            int row = static_cast<int>((pt.z - z_min) / grid_resolution);
            if (col >= 0 && col < cols && row >= 0 && row < rows)
                occupancy[row][col] = true;
        }

        cv::Mat raw_img(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (occupancy[r][c]) {
                    raw_img.at<cv::Vec3b>(rows - 1 - r, c) = cv::Vec3b(0, 0, 255);
                }
            }
        }

        for (const auto& [angle, dist] : angle_distance_map_) {
            float rad = angle * M_PI / 180.0f;
            float x = std::tan(rad) * dist;
            int col = static_cast<int>((x - x_min) / grid_resolution);
            int row = static_cast<int>((dist - z_min) / grid_resolution);
            if (col >= 0 && col < cols && row >= 0 && row < rows) {
                raw_img.at<cv::Vec3b>(rows - 1 - row, col) = cv::Vec3b(0, 255, 255);  // yellow (BGR)
            }
        }

        cv::Mat img;
        cv::resize(raw_img, img, cv::Size(cols * scale, rows * scale), 0, 0, cv::INTER_NEAREST);

        for (int r = 0; r <= rows; ++r) {
            int y = r * scale;
            cv::line(img, cv::Point(0, y), cv::Point(cols * scale, y), cv::Scalar(50, 50, 50), 1);
        }
        for (int c = 0; c <= cols; ++c) {
            int x = c * scale;
            cv::line(img, cv::Point(x, 0), cv::Point(x, rows * scale), cv::Scalar(50, 50, 50), 1);
        }

        int center_col = static_cast<int>((0.0 - x_min) / grid_resolution);
        int center_x = center_col * scale;
        cv::line(img, cv::Point(center_x, 0), cv::Point(center_x, rows * scale), cv::Scalar(0, 255, 0), 2);

        for (int r = 0; r <= rows; r += 5) {
            int z_value = static_cast<int>(z_min + r * grid_resolution);
            int y = (rows - r) * scale - 5;
            cv::putText(img, std::to_string(z_value) + "m", cv::Point(5, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
        }

        cv::imshow("Obstacle Grid View", img);
        cv::waitKey(1);

        auto msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        msg_img->header = msg->header;
        image_pub_->publish(*msg_img);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XZVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}