#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/color/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth/image_raw", 10);

        init_camera();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CameraNode::capture_frame, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rs2::pipeline pipe_;
    rs2::align align_to_color{RS2_STREAM_COLOR};
    rs2::hole_filling_filter hole_filling;  // ✅ 補洞濾波器

    const int width_ = 1280, height_ = 720;

    void init_camera() {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, 30);

        try {
            pipe_.start(cfg);
            auto dev = pipe_.get_active_profile().get_device();
            auto depth_sensor = dev.first<rs2::depth_sensor>();

            // ✅ 設定雷射功率最大值（IR 發射）
            if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
                depth_sensor.set_option(RS2_OPTION_LASER_POWER, 150.f);
            }

            // ✅ 關閉自動曝光
            if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.f);
            }

            // ✅ 強制開啟紅外線投影器
            if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
            }

            // ✅ 增加接收器增益（提升遠距接收）
            if (depth_sensor.supports(RS2_OPTION_DIGITAL_GAIN)) {
                depth_sensor.set_option(RS2_OPTION_DIGITAL_GAIN, 18.f);  // 最大
            }

            // ✅ 設定最大距離（視情況可調整）
            if (depth_sensor.supports(RS2_OPTION_MAX_DISTANCE)) {
                depth_sensor.set_option(RS2_OPTION_MAX_DISTANCE, 15.f);
            }

            // ✅ 使用 Max Range 預設值
            if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
                depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 3);  // 3 = Max Range
            }

            RCLCPP_INFO(this->get_logger(), "RealSense D435i 啟動成功（遠距 + 穩定模式）");

        } catch (const rs2::error &e) {
            RCLCPP_FATAL(this->get_logger(), "相機啟動失敗: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void capture_frame() {
        rs2::frameset frames;
        if (!pipe_.poll_for_frames(&frames)) return;

        rs2::frameset aligned_frames = align_to_color.process(frames);
        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();

        if (color_frame && depth_frame) {
            // ✅ 對深度影像做 hole filling（補洞）
            depth_frame = hole_filling.process(depth_frame);

            cv::Mat color_img(cv::Size(width_, height_), CV_8UC3,
                              (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_img(cv::Size(width_, height_), CV_16UC1,
                              (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_img).toImageMsg();
            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_img).toImageMsg();

            color_msg->header.stamp = depth_msg->header.stamp = this->now();
            color_msg->header.frame_id = depth_msg->header.frame_id = "camera_link";

            color_pub_->publish(*color_msg);
            depth_pub_->publish(*depth_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
