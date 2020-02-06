#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <crs_msgs/srv/load_part.hpp>
#include <crs_msgs/srv/localize_to_part.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_localize_to_part");

  // parameters
  std::vector<std::string> point_cloud_files;
  std::string part_file;
  node->get_parameter("point_cloud_files", point_cloud_files);
  node->get_parameter("part_file", part_file);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds;

  for (auto& pcf : point_cloud_files)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcf, *point_cloud) == -1)
    {
      RCLCPP_ERROR(node->get_logger(), "Couldn't read file %s", pcf.c_str());
      rclcpp::shutdown();
      return 1;
    }

    point_clouds.push_back(point_cloud);
  }

  rclcpp::Client<crs_msgs::srv::LoadPart>::SharedPtr load_part_client =
      node->create_client<crs_msgs::srv::LoadPart>("load_part");
  rclcpp::Client<crs_msgs::srv::LocalizeToPart>::SharedPtr localize_to_part_client =
      node->create_client<crs_msgs::srv::LocalizeToPart>("localize_to_part");

  while (!localize_to_part_client->wait_for_service() || !load_part_client->wait_for_service())
  {
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again.");
  }

  auto load_part_request = std::make_shared<crs_msgs::srv::LoadPart::Request>();
  load_part_request->path_to_part = part_file;

  auto result_future = load_part_client->async_send_request(load_part_request);
  if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    rclcpp::shutdown();
    return 1;
  }
  auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "%d", result->success);

  // pcl::transformPointcloud
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> transform = std::make_pair<Eigen::Vector3d, Eigen::Quaterniond>(
      Eigen::Vector3d(1, 10, 3),
      Eigen::Quaterniond(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())));

  auto localize_to_part_request = std::make_shared<crs_msgs::srv::LocalizeToPart::Request>();
  for (auto& point_cloud : point_clouds)
  {
    pcl::PointCloud<pcl::PointXYZ> tf_point_cloud;
    pcl::transformPointCloud(*point_cloud, tf_point_cloud, transform.first, transform.second);
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(tf_point_cloud, point_cloud_msg);
    localize_to_part_request->point_clouds.push_back(point_cloud_msg);
  }

  auto localize_result_future = localize_to_part_client->async_send_request(localize_to_part_request);
  if (rclcpp::spin_until_future_complete(node, localize_result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    rclcpp::shutdown();
    return 1;
  }
  auto localize_result = localize_result_future.get();
  RCLCPP_INFO(node->get_logger(), "%d", localize_result->success);
  RCLCPP_INFO(node->get_logger(),
              "Transform : (%f, %f, %f)",
              localize_result->transform.transform.translation.x,
              localize_result->transform.transform.translation.y,
              localize_result->transform.transform.translation.z);

  rclcpp::shutdown();
  return 0;
}
