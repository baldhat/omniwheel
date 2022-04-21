#include <librealsense2/rs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

bool enabled = false;

void enable_lidar(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  enabled = request->data;
  response->success = true;
  response->message = enabled ? "Enabled lidar" : "Disabled lidar";
}

class LidarNode : public rclcpp::Node
{
  public:
    rs2::pipeline p;
    rs2::pointcloud pc;
    LidarNode()
    : Node("lidar_node")
    {
      pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("lidar/points", 10);
      enable_service =
        create_service<std_srvs::srv::SetBool>("enable_lidar", &enable_lidar);

      // supported resolutions L515: 320x240, 640x480, 1024x768
      rs2::config cfg;
      cfg.enable_stream(RS2_STREAM_DEPTH, 320, 240, RS2_FORMAT_Z16, 30);
      p.start(cfg);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready...");
    }
    void run() {
      while (true) {
        if (enabled) {
          std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
          rs2::frameset frames = p.wait_for_frames();

          rs2::depth_frame depth = frames.get_depth_frame();

          cloud_.points.clear();
          auto points = pc.calculate(depth);
          auto vertices = points.get_vertices();
          for (size_t i = 0; i < points.size(); i++) {
            pcl::PointXYZ pt = pcl::PointXYZ();
            if (vertices[i].x != 0 || vertices[i].y != 0 || vertices[i].z != 0) {
              pt.x = vertices[i].x;
              pt.y = vertices[i].y;
              pt.z = vertices[i].z;
              cloud_.points.push_back(pt);
            }
          }
          pcl::PCLPointCloud2 tmp_cloud;
          pcl::toPCLPointCloud2(cloud_, tmp_cloud);
          pcl_conversions::fromPCL(tmp_cloud, pc2_msg_);

          pc2_msg_.header.stamp = now();
          pc2_msg_.header.frame_id = "lidar_link";

          pub_->publish(pc2_msg_);

          std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
          std::cout << "FPS = " << 1000000.0 / std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl; 
        }
        rclcpp::spin_some(this->get_node_base_interface());
      }
    }

  private:
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    sensor_msgs::msg::PointCloud2 pc2_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  LidarNode node;
  node.run();
  rclcpp::shutdown();
  return 0;
}
