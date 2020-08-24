#include <crs_perception/model_to_point_cloud.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/segmentation/sac_segmentation.h>

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
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <rclcpp_components/register_node_macro.hpp>

//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
//#define EIGEN_DONT_VECTORIZE

static const double VIEW_POINT_Z = 1000.0;
static const std::string CROP_BOXES_PARAM_PREFIX = "crop_boxes.box";
static const std::string CROP_BOXES_MARKER_NS = "crop_boxes";

struct IcpConfig
{
  bool use_cog_alignment = false;
  bool use_correspondences = false;
  double max_correspondence_dist = 0.01;
  int max_iter = 200;
  double transformation_eps = 1e-2;
  double rotation_eps = 1e-6;
  double euclidean_fitness = 1.0;
  double ransac_threshold = 0.008;
};

struct SACAlignConfig
{
  double normal_est_rad = 0.01;
  double feature_est_rad = 0.025;
  double max_iters = 50000;
  int num_samples = 3;
  int correspondence_rand = 5;
  double similarity_threshold = 0.9;
  double max_correspondence_dist = 0.5;
  double inlier_fraction = 0.25;
};

struct CropBoxConfig
{
  std::vector<double> xyz;
  std::vector<double> size;
  bool reverse;
};

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

visualization_msgs::msg::MarkerArray createMarkers(const std::vector<CropBoxConfig>& configs,
                                                   const std::string& frame_id,
                                                   const std::string& ns)
{
  visualization_msgs::msg::MarkerArray markers;
  for (std::size_t i = 0; i < configs.size(); i++)
  {
    auto& cfg = configs[i];
    Eigen::Isometry3d pose =
        Eigen::Translation3d(Eigen::Vector3d(cfg.xyz[0], cfg.xyz[1], cfg.xyz[2])) * Eigen::Isometry3d::Identity();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pose.translation().x();
    marker.pose.position.y = pose.translation().y();
    marker.pose.position.z = pose.translation().z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = cfg.size[0];
    marker.scale.y = cfg.size[1];
    marker.scale.z = cfg.size[2];

    // setting color
    marker.color.a = 0.1;
    if (cfg.reverse)
    {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }

    markers.markers.push_back(marker);
  }

  return markers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cropBox(const CropBoxConfig& cfg, pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::CropBox<pcl::PointXYZ> box_filter;
  double x_min = cfg.xyz[0] - 0.5 * cfg.size[0];
  double y_min = cfg.xyz[1] - 0.5 * cfg.size[1];
  double z_min = cfg.xyz[2] - 0.5 * cfg.size[2];

  double x_max = cfg.xyz[0] + 0.5 * cfg.size[0];
  double y_max = cfg.xyz[1] + 0.5 * cfg.size[1];
  double z_max = cfg.xyz[2] + 0.5 * cfg.size[2];

  box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  box_filter.setNegative(!cfg.reverse);
  box_filter.setInputCloud(input);
  box_filter.filter(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZ> downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double leaf_size)
{
  using namespace pcl;
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setInputCloud(cloud);
  voxelgrid.setLeafSize(leaf_size, leaf_size, leaf_size);
  pcl::PointCloud<pcl::PointXYZ> out;  // = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxelgrid.filter(out);
  return out;
}

namespace crs_perception
{
class LocalizeToPart : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalizeToPart(const rclcpp::NodeOptions& options)
    : Node("localize_to_part", options)
    , part_loaded_(false)
    , world_frame_("world")
    , clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    , tf_buffer_(clock_)
    , tf_listener_(tf_buffer_)
  {
    using namespace std::chrono_literals;

    part_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!loadParameters())
    {
      throw std::runtime_error("Failed to load parameters");
    }

    // parameter updates subscription
    static const std::string PARAMETER_EVENTS_TOPIC = "parameter_events";
    parameter_event_subs_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        PARAMETER_EVENTS_TOPIC, rclcpp::QoS(1), [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr) {
          loadParameters();
        });

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

    // debug visualization publishers
    if (enable_debug_visualizations_)
    {
      // todo(ayoungs: investigate if this behaves like a latched topic
      loaded_part_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("loaded_part_point_cloud",
                                                                                  rclcpp::QoS(1).transient_local());
      tf_loaded_part_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tf_loaded_part_point_cloud",
                                                                                     rclcpp::QoS(1).transient_local());
      combined_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("combined_scanned_point_clouds",
                                                                               rclcpp::QoS(1).transient_local());

      markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(CROP_BOXES_MARKER_NS, 1);
      markers_timer_ = this->create_wall_timer(500ms, [this]() -> void { markers_pub_->publish(cropbox_markers_); });
    }
  }

private:
  bool loadParameters()
  {
    // general parameters
    mesh_num_samples_ = this->get_parameter("general.mesh_num_samples").as_int();
    leaf_size_ = this->get_parameter("general.leaf_size").as_double();
    part_frame_ = this->get_parameter("general.part_frame").as_string();
    enable_debug_visualizations_ = this->get_parameter("general.enable_debug_visualizations").as_bool();

    // icp parameters
    std::map<std::string, rclcpp::Parameter> params;
    if (this->get_parameters("icp", params))
    {
      icp_config_.use_cog_alignment = params["use_cog_alignment"].as_bool();
      icp_config_.euclidean_fitness = params["euclidean_fitness"].as_double();
      icp_config_.max_correspondence_dist = params["max_correspondence_dist"].as_double();
      icp_config_.max_iter = params["max_iter"].as_int();
      icp_config_.transformation_eps = params["euclidean_fitness"].as_double();
      icp_config_.rotation_eps = params["rotation_eps"].as_double();
      icp_config_.ransac_threshold = params["ransac_threshold"].as_double();
      icp_config_.use_correspondences = params["use_correspondences"].as_bool();
      RCLCPP_INFO_STREAM(this->get_logger(), "Loaded icp parameters");
    }
    else
    {
      std::string err_msg = "Failed to find icp parameters";
      RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
      return false;
    }

    // sac parameters
    params.clear();
    if (this->get_parameters("sac", params))
    {
      sac_align_config_.normal_est_rad = params["normal_est_rad"].as_double();
      sac_align_config_.feature_est_rad = params["feature_est_rad"].as_double();
      sac_align_config_.max_iters = params["max_iters"].as_int();
      sac_align_config_.num_samples = params["num_samples"].as_int();
      sac_align_config_.correspondence_rand = params["correspondence_rand"].as_int();
      sac_align_config_.similarity_threshold = params["similarity_threshold"].as_double();
      sac_align_config_.max_correspondence_dist = params["max_correspondence_dist"].as_double();
      sac_align_config_.inlier_fraction = params["inlier_fraction"].as_double();
      RCLCPP_INFO_STREAM(this->get_logger(), "Loaded sac parameters");
    }
    else
    {
      std::string err_msg = "Failed to find sac parameters";
      RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
      return false;
    }

    // crop boxes
    std::size_t box_counter = 1;
    std::map<std::string, rclcpp::Parameter> crop_box_params;
    std::string cropbox_param_ns = CROP_BOXES_PARAM_PREFIX + std::to_string(box_counter);
    while (this->get_parameters(cropbox_param_ns, crop_box_params))
    {
      CropBoxConfig cb_config;
      cb_config.xyz = crop_box_params["xyz"].as_double_array();
      cb_config.size = crop_box_params["size"].as_double_array();
      cb_config.reverse = crop_box_params["reverse"].as_bool();
      crop_boxes_.push_back(cb_config);
      RCLCPP_INFO(this->get_logger(), "Added cropbox %i", box_counter++);
      cropbox_param_ns = CROP_BOXES_PARAM_PREFIX + std::to_string(box_counter);
      crop_box_params.clear();
    }

    if (crop_boxes_.empty())
    {
      std::string err_msg =
          boost::str(boost::format("Failed to find crop boxes under namespace %s") % CROP_BOXES_PARAM_PREFIX);
      RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
      return false;
    }

    cropbox_markers_ = createMarkers(crop_boxes_, world_frame_, CROP_BOXES_MARKER_NS);

    return true;
  }

  void handleLoadPart(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<crs_msgs::srv::LoadPart::Request> request,
                      const std::shared_ptr<crs_msgs::srv::LoadPart::Response> response)
  {
    namespace fs = boost::filesystem;
    using namespace std::chrono_literals;

    (void)request_header;

    part_loaded_ = false;
    response->success = false;
    if (!fs::exists(fs::path(request->path_to_part)))
    {
      response->error = boost::str(boost::format("Part file %s does not exists") % request->path_to_part);
      RCLCPP_ERROR_STREAM(this->get_logger(), response->error);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading part from %s", request->path_to_part.c_str());

    // converting mesh into cloud
    ModelToPointCloud mtpc(mesh_num_samples_, leaf_size_);
    if (!mtpc.convertToPCL(request->path_to_part, part_point_cloud_, response->error))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), response->error);
      return;
    }

    // transforming point cloud
    Eigen::Isometry3d part_seed_transform_ = Eigen::Isometry3d::Identity();
    part_seed_msg_ = request->part_origin;
    tf2::fromMsg(part_seed_msg_, part_seed_transform_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud = part_point_cloud_->makeShared();
    pcl::transformPointCloud(*original_cloud, *part_point_cloud_, part_seed_transform_.matrix());
    part_loaded_ = true;
    response->success = true;

    if (enable_debug_visualizations_)
    {
      publish_timer_.reset();

      sensor_msgs::msg::PointCloud2 point_cloud;
      pcl::toROSMsg(*part_point_cloud_, point_cloud);
      point_cloud.header.stamp = this->now();
      point_cloud.header.frame_id = world_frame_;

      sensor_msgs::msg::PointCloud2 empty_cloud;
      empty_cloud.header.stamp = this->now();
      empty_cloud.header.frame_id = world_frame_;

      publish_timer_ = this->create_wall_timer(500ms, [this, point_cloud, empty_cloud]() -> void {
        loaded_part_pc_pub_->publish(point_cloud);
        combined_pc_pub_->publish(empty_cloud);
      });
    }
  }

  Eigen::Isometry3d findTransform(const pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                  const bool pos_only = true)
  {
    using namespace Eigen;
    auto compute_centroid_transform = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) -> Eigen::Isometry3d {
      Eigen::Vector3f center, x_axis, y_axis, z_axis;
      pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moi;
      moi.setInputCloud(cloud);
      moi.compute();
      moi.getEigenVectors(x_axis, y_axis, z_axis);
      moi.getMassCenter(center);

      Eigen::Isometry3d transform;
      transform.setIdentity();
      transform.translation() = center.cast<double>();
      transform.linear().col(0) = x_axis.normalized().cast<double>();
      transform.linear().col(1) = y_axis.normalized().cast<double>();
      transform.linear().col(2) = z_axis.normalized().cast<double>();
      return transform;
    };

    Eigen::Isometry3d src_transform = compute_centroid_transform(src_cloud);
    Eigen::Isometry3d target_transform = compute_centroid_transform(target_cloud);
    Eigen::Isometry3d pose = target_transform * src_transform.inverse();
    if (pos_only)
    {
      pose.linear() = Quaterniond::Identity().toRotationMatrix().matrix();
    }

    Eigen::Vector3d angles = pose.linear().eulerAngles(0, 1, 2);
    RCLCPP_INFO_STREAM(this->get_logger(), "Euler angles: " << angles.transpose());
    return pose;
  }

  bool alignIcpNormals(Cloud::Ptr target_cloud, Cloud::Ptr src_cloud, Eigen::Isometry3d& transform)
  {
    using PointNT = pcl::PointNormal;
    using CloudN = pcl::PointCloud<PointNT>;

    // initial alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Isometry3d init_transform = findTransform(src_cloud, target_cloud, false);
    pcl::transformPointCloud(*src_cloud, *temp_cloud, init_transform.cast<float>());
    *src_cloud = *temp_cloud;

    CloudN::Ptr src_normals = boost::make_shared<CloudN>();
    CloudN::Ptr target_normals = boost::make_shared<CloudN>();

    // copy cloud data
    pcl::copyPointCloud(*src_cloud, *src_normals);
    pcl::copyPointCloud(*target_cloud, *target_normals);

    // Estimate normals
    pcl::console::print_highlight("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setViewPoint(0.0, 0.0, VIEW_POINT_Z);
    nest.setRadiusSearch(sac_align_config_.normal_est_rad);
    nest.setInputCloud(src_normals);
    nest.compute(*src_normals);
    nest.setInputCloud(target_normals);
    nest.compute(*target_normals);

    // icp
    pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp;
    icp.setUseReciprocalCorrespondences(icp_config_.use_correspondences);
    icp.setMaxCorrespondenceDistance(icp_config_.max_correspondence_dist);
    icp.setMaximumIterations(icp_config_.max_iter);
    icp.setTransformationEpsilon(icp_config_.transformation_eps);
    // TODO: Uncomment this after migrating to pcl 1.9.1
    // //icp.setTransformationRotationEpsilon(icp_config_.rotation_eps);
    icp.setEuclideanFitnessEpsilon(icp_config_.euclidean_fitness);
    icp.setRANSACOutlierRejectionThreshold(icp_config_.ransac_threshold);
    icp.setInputSource(src_normals);
    icp.setInputTarget(target_normals);
    pcl::PointCloud<PointNT> final;
    icp.align(final);

    if (!icp.hasConverged())
    {
      return false;
    }

    transform.matrix() = icp.getFinalTransformation().cast<double>();
    transform = init_transform * transform;
    return true;
  }

  bool alignIcp(Cloud::Ptr target_cloud, Cloud::Ptr src_cloud, Eigen::Isometry3d& transform)
  {
    Eigen::Isometry3d init_transform = Eigen::Isometry3d::Identity();
    if (icp_config_.use_cog_alignment)
    {
      // initial alignment
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      init_transform = findTransform(src_cloud, target_cloud, false);
      pcl::transformPointCloud(*src_cloud, *temp_cloud, init_transform.cast<float>());
      *src_cloud = *temp_cloud;
    }

    // icp
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setUseReciprocalCorrespondences(icp_config_.use_correspondences);
    icp.setMaxCorrespondenceDistance(icp_config_.max_correspondence_dist);
    icp.setMaximumIterations(icp_config_.max_iter);
    icp.setTransformationEpsilon(icp_config_.transformation_eps);
    // TODO: Uncomment this after migrating to pcl 1.9.1
    // icp.setTransformationRotationEpsilon(icp_config_.rotation_eps);
    icp.setEuclideanFitnessEpsilon(icp_config_.euclidean_fitness);
    icp.setRANSACOutlierRejectionThreshold(icp_config_.ransac_threshold);
    icp.setInputSource(src_cloud);
    icp.setInputTarget(target_cloud);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);

    if (!icp.hasConverged())
    {
      return false;
    }

    transform.matrix() = icp.getFinalTransformation().cast<double>();
    transform = init_transform * transform;
    return true;
  }

  /**
   * @brief aligs a point cloud using the SampleConsensusPrerejective algorithm, see
   * https://pcl-tutorials.readthedocs.io/en/latest/alignment_prerejective.html#alignment-prerejective
   */
  bool alignSac(Cloud::Ptr target_cloud, Cloud::Ptr src_cloud, Eigen::Isometry3d& transform)
  {
    using PointNT = pcl::PointNormal;
    using CloudN = pcl::PointCloud<PointNT>;
    using FeatureT = pcl::FPFHSignature33;
    using FeatureCloudT = pcl::PointCloud<FeatureT>;
    using FeatureEstimationT = pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT>;

    CloudN::Ptr object = boost::make_shared<CloudN>();
    CloudN::Ptr object_aligned = boost::make_shared<CloudN>();
    CloudN::Ptr scene = boost::make_shared<CloudN>();
    FeatureCloudT::Ptr object_features = boost::make_shared<FeatureCloudT>();
    FeatureCloudT::Ptr scene_features = boost::make_shared<FeatureCloudT>();

    // initial alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Isometry3d init_transform = findTransform(src_cloud, target_cloud, true);
    pcl::transformPointCloud(*src_cloud, *temp_cloud, init_transform.cast<float>());
    *src_cloud = *temp_cloud;

    // copy cloud data
    pcl::copyPointCloud(*src_cloud, *object);
    pcl::copyPointCloud(*target_cloud, *scene);

    // Estimate normals for scene
    pcl::console::print_highlight("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setViewPoint(0.0, 0.0, VIEW_POINT_Z);
    nest.setRadiusSearch(sac_align_config_.normal_est_rad);
    nest.setInputCloud(scene);
    nest.compute(*scene);

    // Estimate normals for object
    pcl::console::print_highlight("Estimating object normals...\n");
    nest.setInputCloud(object);
    nest.compute(*object);

    // Estimate features
    pcl::console::print_highlight("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch(sac_align_config_.feature_est_rad);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*object_features);
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*scene_features);

    // Perform alignment
    pcl::console::print_highlight("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
    align.setInputSource(object);
    align.setSourceFeatures(object_features);
    align.setInputTarget(scene);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(sac_align_config_.max_iters);  // Number of RANSAC iterations
    align.setNumberOfSamples(sac_align_config_.num_samples);  // Number of points to sample for generating/prerejecting
                                                              // a pose
    align.setCorrespondenceRandomness(sac_align_config_.correspondence_rand);  // Number of nearest features to use
    align.setSimilarityThreshold(sac_align_config_.similarity_threshold);  // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(sac_align_config_.max_correspondence_dist);  // Inlier threshold
    align.setInlierFraction(sac_align_config_.inlier_fraction);  // Required inlier fraction for accepting a pose
                                                                 // hypothesis
    {
      pcl::ScopeTime t("Alignment");
      align.align(*object_aligned);
    }

    if (!align.hasConverged())
    {
      return false;
    }

    transform.setIdentity();
    transform.matrix() = align.getFinalTransformation().cast<double>();
    transform = transform * init_transform;
    return true;
  }

  void handleLocalizeToPart(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<crs_msgs::srv::LocalizeToPart::Request> request,
                            const std::shared_ptr<crs_msgs::srv::LocalizeToPart::Response> response)
  {
    using namespace std::chrono_literals;

    (void)request_header;

    if (enable_debug_visualizations_)
    {
      publish_timer_.reset();

      sensor_msgs::msg::PointCloud2 loaded_cloud;
      pcl::toROSMsg(*part_point_cloud_, loaded_cloud);
      loaded_cloud.header.stamp = this->now();
      loaded_cloud.header.frame_id = world_frame_;

      sensor_msgs::msg::PointCloud2 empty_cloud;
      empty_cloud.header.stamp = this->now();
      empty_cloud.header.frame_id = world_frame_;

      publish_timer_ = this->create_wall_timer(500ms, [this, loaded_cloud, empty_cloud]() -> void {
        loaded_part_pc_pub_->publish(loaded_cloud);
        combined_pc_pub_->publish(empty_cloud);
      });
    }

    if (!part_loaded_)
    {
      response->success = false;
      response->error = "Missing part. Please load a part first.";
      RCLCPP_ERROR_STREAM(this->get_logger(), response->error);
      return;
    }

    // getting part seed transform
    Eigen::Isometry3d part_seed_transform_;
    tf2::fromMsg(part_seed_msg_, part_seed_transform_);

    // TODO: better input verification
    if (request->point_clouds.empty() && request->transforms.size() != request->point_clouds.size())
    {
      response->success = false;
      response->error = "Request has invalid data";
      RCLCPP_ERROR_STREAM(this->get_logger(), response->error);
      return;
    }

    // combining all clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (std::size_t i = 0; i < request->point_clouds.size(); i++)
    {
      auto& point_cloud_msg = request->point_clouds[i];
      geometry_msgs::msg::TransformStamped& transform = request->transforms[i];

      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

      pcl::fromROSMsg(point_cloud_msg, *point_cloud);
      pcl::transformPointCloud(*point_cloud, *transformed_cloud, tf2::transformToEigen(transform).matrix());

      *combined_point_cloud += *transformed_cloud;
    }

    RCLCPP_INFO(this->get_logger(), "Combined point cloud has %lu points", combined_point_cloud->size());

    // transforming to requested frame
    std::string src_frame_id = request->transforms.front().header.frame_id;
    if (src_frame_id != request->frame)
    {
      try
      {
        RCLCPP_INFO(
            this->get_logger(), "Transforming cloud from frame %s to %s", src_frame_id.c_str(), request->frame.c_str());
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_.lookupTransform(request->frame, src_frame_id, tf2::TimePointZero);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        ;
        pcl::transformPointCloud(*combined_point_cloud, *transformed_cloud, tf2::transformToEigen(transform).matrix());
        *combined_point_cloud = *transformed_cloud;
      }
      catch (tf2::TransformException ex)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        response->success = false;
        response->error =
            "Failed to transform point cloud from '" + src_frame_id + "' to '" + request->frame + "' frame";
        return;
      }
    }

    // cropping
    for (auto& cfg : crop_boxes_)
    {
      combined_point_cloud = cropBox(cfg, combined_point_cloud);
    }
    RCLCPP_INFO(this->get_logger(), "Combined point cloud has %lu points after cropping", combined_point_cloud->size());

    if (combined_point_cloud->empty())
    {
      response->success = false;
      response->error = "No point remains after cropping";
      RCLCPP_ERROR_STREAM(this->get_logger(), response->error);
      return;
    }

    if (enable_debug_visualizations_)
    {
      publish_timer_.reset();

      sensor_msgs::msg::PointCloud2 scanned_cloud;
      pcl::toROSMsg(*combined_point_cloud, scanned_cloud);
      scanned_cloud.header.stamp = this->now();
      scanned_cloud.header.frame_id = world_frame_;

      sensor_msgs::msg::PointCloud2 loaded_cloud;
      pcl::toROSMsg(*part_point_cloud_, loaded_cloud);
      loaded_cloud.header.stamp = this->now();
      loaded_cloud.header.frame_id = world_frame_;

      publish_timer_ = this->create_wall_timer(500ms, [this, loaded_cloud, scanned_cloud]() -> void {
        loaded_part_pc_pub_->publish(loaded_cloud);
        combined_pc_pub_->publish(scanned_cloud);
      });
    }

    // downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    *src_cloud = downsampleCloud(part_point_cloud_, leaf_size_);
    *target_cloud = downsampleCloud(combined_point_cloud, leaf_size_);
    RCLCPP_INFO(this->get_logger(), "Combined point cloud has %lu points after downsampling", target_cloud->size());

    // aligning
    Eigen::Isometry3d transform;
    response->success = alignIcp(target_cloud, src_cloud, transform);
    RCLCPP_INFO_EXPRESSION(this->get_logger(), response->success, "ICP converged");
    RCLCPP_ERROR_EXPRESSION(this->get_logger(), !response->success, "ICP failed to converge");

    // printing transform
    Eigen::Vector3d angles = transform.linear().eulerAngles(0, 1, 2);
    RCLCPP_INFO(this->get_logger(),
                "Transform : p(%f, %f, %f), r(%f, %f, %f)",
                response->transform.transform.translation.x,
                response->transform.transform.translation.y,
                response->transform.transform.translation.z,
                angles(0),
                angles(1),
                angles(2));

    response->transform = tf2::eigenToTransform(transform * part_seed_transform_);  // applying seed
    response->transform.header.stamp = this->now();
    response->transform.header.frame_id = request->frame;
    response->transform.child_frame_id = part_frame_;

    if (enable_debug_visualizations_)
    {
      publish_timer_.reset();

      sensor_msgs::msg::PointCloud2 scanned_cloud;
      pcl::toROSMsg(*combined_point_cloud, scanned_cloud);
      scanned_cloud.header.stamp = this->now();
      scanned_cloud.header.frame_id = world_frame_;

      pcl::transformPointCloud(*part_point_cloud_, *src_cloud, transform.cast<float>());
      sensor_msgs::msg::PointCloud2 registered_cloud;
      pcl::toROSMsg(*src_cloud, registered_cloud);
      registered_cloud.header.stamp = this->now();
      registered_cloud.header.frame_id = world_frame_;

      publish_timer_ = this->create_wall_timer(500ms, [this, registered_cloud, scanned_cloud]() -> void {
        loaded_part_pc_pub_->publish(registered_cloud);
        combined_pc_pub_->publish(scanned_cloud);
      });
    }
  }

  // parameters
  int mesh_num_samples_;
  double leaf_size_;
  bool enable_debug_visualizations_;
  std::string part_frame_;
  IcpConfig icp_config_;
  SACAlignConfig sac_align_config_;
  std::vector<CropBoxConfig> crop_boxes_;

  // subscriber parameters
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subs_;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_part_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tf_loaded_part_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // services
  rclcpp::Service<crs_msgs::srv::LoadPart>::SharedPtr load_part_service_;
  rclcpp::Service<crs_msgs::srv::LocalizeToPart>::SharedPtr localize_to_part_service_;

  // timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr markers_timer_;

  // data
  geometry_msgs::msg::Pose part_seed_msg_;
  bool part_loaded_;
  std::string world_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr part_point_cloud_;
  visualization_msgs::msg::MarkerArray cropbox_markers_;

  // other
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace crs_perception

RCLCPP_COMPONENTS_REGISTER_NODE(crs_perception::LocalizeToPart)

int main(int argc, char** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  static const rclcpp::Logger LOGGER = rclcpp::get_logger("localize_to_part");

  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<crs_perception::LocalizeToPart> node = std::make_shared<crs_perception::LocalizeToPart>(node_options);

  rclcpp::spin(node);
  return 0;
}
