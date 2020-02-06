#include <crs_perception/model_to_point_cloud.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <crs_msgs/srv/load_part.hpp>
#include <crs_msgs/srv/localize_to_part.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crs_perception
{
class LocalizeToPart : public rclcpp::Node
{
public:
  LocalizeToPart(const rclcpp::NodeOptions& options) : rclcpp::Node("localize_to_part", options), part_loaded_(false)
  {
    part_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    // parameters
    mesh_num_samples_ = this->declare_parameter("mesh_num_samples", 100000);
    leaf_size_ = this->declare_parameter("leaf_size", 1.0);
    part_frame_ = this->declare_parameter("part_frame", "/part");

    // services
    load_part_service_ = this->create_service<crs_msgs::srv::LoadPart>("load_part",
                                                                       std::bind(&LocalizeToPart::handleLoadPart,
                                                                                 this,
                                                                                 std::placeholders::_1,
                                                                                 std::placeholders::_2,
                                                                                 std::placeholders::_3));
    localize_to_part_service_ =
        this->create_service<crs_msgs::srv::LocalizeToPart>("localize_to_part",
                                                            std::bind(&LocalizeToPart::handleLocalizeToPart,
                                                                      this,
                                                                      std::placeholders::_1,
                                                                      std::placeholders::_2,
                                                                      std::placeholders::_3));
  }

private:
  // parameters
  int mesh_num_samples_;
  double leaf_size_;
  std::string part_frame_;

  // services
  rclcpp::Service<crs_msgs::srv::LoadPart>::SharedPtr load_part_service_;
  rclcpp::Service<crs_msgs::srv::LocalizeToPart>::SharedPtr localize_to_part_service_;

  bool part_loaded_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr part_point_cloud_;

  void handleLoadPart(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<crs_msgs::srv::LoadPart::Request> request,
                      const std::shared_ptr<crs_msgs::srv::LoadPart::Response> response)
  {
    (void)request_header;
    RCLCPP_INFO(get_logger(), "Loading part from %s", request->path_to_part.c_str());

    // todo(ayoungs): handle error for bad file
    // todo(ayoungs): make sure part gets loaded correctly after multiple loads
    ModelToPointCloud mtpc(request->path_to_part, mesh_num_samples_, leaf_size_);
    mtpc.convertToPCL(part_point_cloud_);

    part_loaded_ = true;
    response->success = true;

    // todo(ayoungs): should this publish the pointcloud so that other nodes know the original pose or is it assumed to
    // use the same origin as the CAD file?
  }

  void handleLocalizeToPart(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<crs_msgs::srv::LocalizeToPart::Request> request,
                            const std::shared_ptr<crs_msgs::srv::LocalizeToPart::Response> response)
  {
    (void)request_header;
    if (part_loaded_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr combined_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
      for (auto& point_cloud_msg : request->point_clouds)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

        pcl::fromROSMsg(point_cloud_msg, *point_cloud);
        *combined_point_cloud += *point_cloud;
      }

      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputSource(combined_point_cloud);
      icp.setInputTarget(part_point_cloud_);
      pcl::PointCloud<pcl::PointXYZ> final;
      icp.align(final);
      // todo(ayoungs): use convergence and fitness score?
      // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      // icp.getFitnessScore() << std::endl;

      // todo(ayoungs): should this generate the timestamp or use one of the point cloud stamps
      response->transform.header.stamp = this->now();
      response->transform.header.frame_id = request->frame;
      response->transform.child_frame_id = part_frame_;

      // todo(ayoungs): look for EigenToTf for 4x4; it may already exist
      Eigen::Matrix4d tm = icp.getFinalTransformation().cast<double>();
      tf2::Transform tf2_transform(
          tf2::Matrix3x3(tm(0, 0), tm(0, 1), tm(0, 2), tm(1, 0), tm(1, 1), tm(1, 2), tm(2, 0), tm(2, 1), tm(2, 2)),
          tf2::Vector3(tm(0, 3), tm(1, 3), tm(2, 3)));
      response->transform.transform = tf2::toMsg(tf2_transform);

      response->success = true;
    }
    else
    {
      response->success = true;
      response->error = "Missing part. Please load a part first.";
    }
  }
};

}  // namespace crs_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crs_perception::LocalizeToPart)
