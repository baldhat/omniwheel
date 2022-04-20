#include <librealsense2/rs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class LidarNode : public rclcpp::Node
{
  public:
    rs2::pipeline p;
    rs2::pointcloud pc;
    LidarNode()
    : Node("lidar_node")
    {
      pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("lidar/points", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&LidarNode::timer_callback, this));
      p.start();
    }

  private:
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    sensor_msgs::msg::PointCloud2 pc2_msg_;

    void timer_callback() {
      rs2::frameset frames = p.wait_for_frames();

      rs2::depth_frame depth = frames.get_depth_frame();

      auto points = pc.calculate(depth);
      auto vertices = points.get_vertices();
      for (int i = 0; i < points.size(); i++) {
        pcl::PointXYZ pt = pcl::PointXYZ();
	pt.x = vertices[i].x;
	pt.y = vertices[i].y;
	pt.z = vertices[i].z;
	cloud_.points.push_back(pt);
      }
      pcl::PCLPointCloud2 tmp_cloud;
      pcl::toPCLPointCloud2(cloud_, tmp_cloud);
      pcl_conversions::fromPCL(tmp_cloud, pc2_msg_);

      pc2_msg_.header.stamp = now();
      pc2_msg_.header.frame_id = "lidar_link";
      pub_->publish(pc2_msg_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
