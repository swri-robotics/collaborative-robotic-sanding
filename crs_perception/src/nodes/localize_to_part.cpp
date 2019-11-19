#include <crs_perception/model_to_point_cloud.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <crs_msgs/srv/load_part.hpp>
#include <crs_msgs/srv/localize_to_part.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crs_perception
{

  class LocalizeToPart : public rclcpp::Node
  {
  public:
    LocalizeToPart(const rclcpp::NodeOptions& options) :
      rclcpp::Node("localize_to_part", options),
      part_loaded_(false)
    {
      part_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

      // parameters
      this->get_parameter_or("mesh_num_samples", mesh_num_samples_, 100000);
      this->get_parameter_or("leaf_size", leaf_size_, 0.01);

      // services
      this->create_service<crs_msgs::srv::LoadPart>(
        "load_part", 
        std::bind(&LocalizeToPart::handleLoadPart, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
      this->create_service<crs_msgs::srv::LocalizeToPart>(
        "localize_to_part", 
        std::bind(&LocalizeToPart::handleLocalizeToPart, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
    }

  private:
    // parameters
    int mesh_num_samples_;
    double leaf_size_;

    bool part_loaded_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr part_point_cloud_;

    void handleLoadPart(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<crs_msgs::srv::LoadPart::Request> request,
      const std::shared_ptr<crs_msgs::srv::LoadPart::Response> response)
    {
      (void)request_header;

      // todo(ayoungs): handle error for bad file
      // todo(ayoungs): make sure part gets loaded correctly after multiple loads
      ModelToPointCloud mtpc(request->path_to_part, mesh_num_samples_, leaf_size_);
      mtpc.convertToPCL(part_point_cloud_);

      part_loaded_ = true;
      response->success = true;
    }

    void handleLocalizeToPart(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<crs_msgs::srv::LocalizeToPart::Request> request,
      const std::shared_ptr<crs_msgs::srv::LocalizeToPart::Response> response)
    {
      (void)request_header;
      if (part_loaded_)
      {
        for (auto &ros_point_cloud2: request->point_clouds)
        {
          pcl::PCLPointCloud2 point_cloud2;
          pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
          pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

          // todo(ayoungs): this seems like a lot of copying
          pcl_conversions::toPCL(ros_point_cloud2, point_cloud2);
          pcl::fromPCLPointCloud2(point_cloud2, *point_cloud);

          icp.setInputSource(point_cloud);
          icp.setInputTarget(part_point_cloud_);
          pcl::PointCloud<pcl::PointXYZ> final;
          icp.align(final);
          // todo(ayoungs): use convergence and fitness score?
          // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
          // icp.getFitnessScore() << std::endl;
          geometry_msgs::msg::TransformStamped transform;
          icp.getFinalTransformation();
          response->transforms.push_back(transform);
        }
      }
    }
  };

} // namespace crs_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crs_perception::LocalizeToPart)