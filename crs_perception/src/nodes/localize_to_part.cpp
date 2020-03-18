#include <crs_perception/model_to_point_cloud.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <crs_msgs/srv/load_part.hpp>
#include <crs_msgs/srv/localize_to_part.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

//delet me
#include <pcl/io/pcd_io.h>

namespace crs_perception
{
class LocalizeToPart : public rclcpp::Node
{
public:
  LocalizeToPart(const rclcpp::NodeOptions& options)
    : Node("localize_to_part", options)
    , part_loaded_(false)
    , world_frame_("world")
    , clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    , tf_buffer_(clock_)
    , tf_listener_(tf_buffer_)
  {
    part_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    // parameters
    mesh_num_samples_ = this->declare_parameter("mesh_num_samples", 100000);
    leaf_size_ = this->declare_parameter("leaf_size", 0.01);
    part_frame_ = this->declare_parameter("part_frame", "part");
    enable_debug_visualizations_ = this->declare_parameter("enable_debug_visualizations", true);

    // debug visualization publishers
    if (enable_debug_visualizations_)
    {
      // todo(ayoungs: investigate if this behaves like a latched topic
      loaded_part_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("loaded_part_point_cloud", rclcpp::QoS(1).transient_local());
      tf_loaded_part_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tf_loaded_part_point_cloud", rclcpp::QoS(1).transient_local());
      combined_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("combined_scanned_point_clouds", rclcpp::QoS(1).transient_local());
    }

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

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_part_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tf_loaded_part_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_pub_;

  // services
  rclcpp::Service<crs_msgs::srv::LoadPart>::SharedPtr load_part_service_;
  rclcpp::Service<crs_msgs::srv::LocalizeToPart>::SharedPtr localize_to_part_service_;

  bool part_loaded_;
  std::string world_frame_;
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr part_point_cloud_;

  bool enable_debug_visualizations_;

  void handleLoadPart(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<crs_msgs::srv::LoadPart::Request> request,
                      const std::shared_ptr<crs_msgs::srv::LoadPart::Response> response)
  {
    namespace fs = boost::filesystem;
    (void)request_header;

    part_loaded_ = false;
    response->success = false;
    if(!fs::exists(fs::path(request->path_to_part)))
    {
      response->error = boost::str(boost::format("Part file %s does not exists") % request->path_to_part);
      RCLCPP_ERROR_STREAM(this->get_logger(),response->error);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading part from %s", request->path_to_part.c_str());

    ModelToPointCloud mtpc(mesh_num_samples_, leaf_size_);
    if(!mtpc.convertToPCL(request->path_to_part,part_point_cloud_, response->error))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),response->error);
      return;
    }

    part_loaded_ = true;
    response->success = true;

    if (enable_debug_visualizations_)
    {
      sensor_msgs::msg::PointCloud2 point_cloud;
      pcl::toROSMsg(*part_point_cloud_, point_cloud);
      point_cloud.header.stamp = this->now();
      point_cloud.header.frame_id = world_frame_;
      loaded_part_pc_pub_->publish(point_cloud);

      pcl::PCDWriter w;
      w.writeBinaryCompressed("/home/ayoungs/test_loaded_part.pcd", *part_point_cloud_);
    }
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
        geometry_msgs::msg::TransformStamped transform;
        try
        {
          //transform = tf_buffer_.lookupTransform(world_frame_, point_cloud_msg.header.frame_id, point_cloud_msg.header.stamp);
          transform = tf_buffer_.lookupTransform(world_frame_, point_cloud_msg.header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException ex)
        {
          RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
          response->success = false;
          response->error = "Failed to transform point cloud from '" + point_cloud_msg.header.frame_id + "' to '" + world_frame_ + "' frame";
          return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

        pcl::fromROSMsg(point_cloud_msg, *point_cloud);
        pcl::transformPointCloud(*point_cloud, *transformed_cloud, 
                                 tf2::transformToEigen(transform).matrix());

        *combined_point_cloud += *transformed_cloud;
      }

      if (enable_debug_visualizations_)
      {
        sensor_msgs::msg::PointCloud2 point_cloud;
        pcl::toROSMsg(*combined_point_cloud, point_cloud);
        point_cloud.header.stamp = this->now();
        point_cloud.header.frame_id = world_frame_;
        combined_pc_pub_->publish(point_cloud);

        pcl::PCDWriter w;
        w.writeBinaryCompressed("/home/ayoungs/test_loaded_part.pcd", *part_point_cloud_);
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

      RCLCPP_INFO(this->get_logger(),
                  "Transform : (%f, %f, %f)",
                  response->transform.transform.translation.x,
                  response->transform.transform.translation.y,
                  response->transform.transform.translation.z);

      response->success = true;
    }
    else
    {
      response->success = false;
      response->error = "Missing part. Please load a part first.";
    }
  }
};

}  // namespace crs_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crs_perception::LocalizeToPart)
