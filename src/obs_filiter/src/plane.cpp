#include <memory>
#include <mutex>
#include <random>
#include <numeric>
#include <optional>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>

class PlaneNode : public rclcpp::Node {
public:
    PlaneNode() : Node("plane_node") {
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", 10, std::bind(&PlaneNode::color_callback, this, std::placeholders::_1));
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/image_raw", 10, std::bind(&PlaneNode::depth_callback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_cloud", 10);
        coeff_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/plane_coeff", 10);

        RCLCPP_INFO(this->get_logger(), "PlaneNode started with ROI filtering and efficient plane fitting");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_, depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr coeff_pub_;

    std::mutex mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_color_, latest_depth_;
    std::optional<std::vector<float>> last_valid_coeff_;

    const float fx_ = 904.731f, fy_ = 904.320f, cx_ = 640.644f, cy_ = 362.315f;
    int frame_skip_count_ = 0;
    const size_t max_sample_points_ = 10000;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> random_sample_cloud(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& input_cloud, size_t max_points) {
        auto sampled = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        size_t N = input_cloud->size();
        if (N <= max_points) return input_cloud;

        std::vector<int> indices(N);
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});

        for (size_t i = 0; i < max_points; ++i)
            sampled->push_back(input_cloud->points[indices[i]]);

        return sampled;
    }

    void color_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_color_ = msg;
        try_process();
    }

    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_depth_ = msg;
        try_process();
    }

    void try_process() {
        if (!latest_color_ || !latest_depth_) return;
        if (++frame_skip_count_ % 1 != 0) return;

        auto color_msg = latest_color_;
        auto depth_msg = latest_depth_;
        latest_color_.reset();
        latest_depth_.reset();

        cv::Mat color, depth;
        try {
            color = cv_bridge::toCvCopy(color_msg, "bgr8")->image;
            depth = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed.");
            return;
        }

        if (color.empty() || depth.empty() || color.size() != depth.size()) return;

        auto full_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        auto roi_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        const int rows = depth.rows;
        const int cols = depth.cols;

        for (int y = 0; y < rows; y += 4) {
            const uint16_t* depth_row = depth.ptr<uint16_t>(y);
            const cv::Vec3b* color_row = color.ptr<cv::Vec3b>(y);
            for (int x = 0; x < cols; x += 4) {
                uint16_t d = depth_row[x];
                if (d == 0) continue;

                float z = d / 1000.0f;
                if (z < 0.1f || z > 8.0f || std::isnan(z)) continue;

                float x3d = (x - cx_) * z / fx_;
                float y3d = (y - cy_) * z / fy_;

                pcl::PointXYZRGB pt;
                pt.x = x3d; pt.y = y3d; pt.z = z;
                pt.b = color_row[x][0];
                pt.g = color_row[x][1];
                pt.r = color_row[x][2];

                full_cloud->push_back(pt);
                if (y >= rows * 0.5 && y <= rows * 0.9 && y3d > -0.1f && y3d < 0.3f)
                    roi_cloud->push_back(pt);
            }
        }

        if (roi_cloud->size() >= 50) {
            auto downsampled = random_sample_cloud(roi_cloud, max_sample_points_);

            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(500);
            seg.setDistanceThreshold(0.025);
            seg.setInputCloud(downsampled);

            try {
                seg.segment(*inliers, *coeff);
                float nx = coeff->values[0], ny = coeff->values[1], nz = coeff->values[2];
                float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
                if (norm == 0) return;
                nx /= norm; ny /= norm; nz /= norm;

                bool is_ground_plane = ny < -0.8 && std::abs(nx) < 0.2 && std::abs(nz) < 0.6;

                if (!is_ground_plane) {
                    RCLCPP_WARN(this->get_logger(), "Plane normal not ground-aligned: [%.2f %.2f %.2f] → using previous", nx, ny, nz);
                    if (last_valid_coeff_) {
                        coeff->values = *last_valid_coeff_;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "No previous valid plane available, skipping publish");
                        return;
                    }
                } else {
                    last_valid_coeff_ = coeff->values;
                }

                auto inlier_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                for (int idx : inliers->indices) {
                    if (idx >= 0 && idx < static_cast<int>(downsampled->size())) {
                        pcl::PointXYZRGB pt = downsampled->points[idx];
                        pt.r = 0; pt.g = 255; pt.b = 0;
                        inlier_cloud->push_back(pt);
                    }
                }
                *full_cloud += *inlier_cloud;

                std_msgs::msg::Float32MultiArray coeff_msg;
                coeff_msg.data = coeff->values;
                coeff_pub_->publish(coeff_msg);

                RCLCPP_INFO(this->get_logger(), "Plane fitted: coeff = [%.3f %.3f %.3f %.3f]", coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Segmentation error: %s", e.what());
            }
        }

        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*full_cloud, ros_cloud);
        ros_cloud.header.stamp = color_msg->header.stamp;
        ros_cloud.header.frame_id = "camera_link";
        cloud_pub_->publish(ros_cloud);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Published full cloud: %zu pts", full_cloud->size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneNode>());
    rclcpp::shutdown();
    return 0;
}