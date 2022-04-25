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
const int CAPACITY = 5; // allow max latency of 5 frames
rs2::frame_queue queue(CAPACITY);

void enable_lidar(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  enabled = request->data;
  response->success = true;
  response->message = enabled ? "Enabled" : "Disabled";
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
        //cfg.enable_stream(RS2_STREAM_INFRARED, 320, 240, RS2_FORMAT_Z16, 30);
        p.start(cfg);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready...");
    }
    rs2::depth_frame frame;
    void run() {
      std::thread t([&]() {
        while (true){
            if (queue.poll_for_frame(&frame)) {
                auto depth = frame.get_data();
                cloud_.points.clear();
                auto points = pc.calculate(depth);
                auto vertices = points.get_vertices();
                for (size_t i = 0; i < points.size(); i++) {
                    pcl::PointXYZI pt = pcl::PointXYZI();
                    if (vertices[i].x != 0 || vertices[i].y != 0 || vertices[i].z != 0) {
                        pt.x = vertices[i].x;
                        pt.y = vertices[i].y;
                        pt.z = vertices[i].z;
                        pt.intensity = 1.0;
                        cloud_.points.push_back(pt);
                    }
                }
                pcl::PCLPointCloud2 tmp_cloud;
                pcl::toPCLPointCloud2(cloud_, tmp_cloud);
                pcl_conversions::fromPCL(tmp_cloud, pc2_msg_);

                pc2_msg_.header.stamp = now();
                pc2_msg_.header.frame_id = "lidar_link";

                pub_->publish(pc2_msg_);
            }
        }
      });
      t.detach();

      while (true) {
        if (enabled) {
          rs2::frameset frames = p.wait_for_frames();
          queue.enqueue(frames.get_depth_frame());
        }
        rclcpp::spin_some(this->get_node_base_interface());
      }
    }

  private:
    pcl::PointCloud<pcl::PointXYZI> cloud_;
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
