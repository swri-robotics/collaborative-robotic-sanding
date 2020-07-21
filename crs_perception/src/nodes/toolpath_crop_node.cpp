/*
 * toolpath_crop_node.cpp
 *
 * Created on: Jul 20, 2020
 * Author: Jorge Nicho
 *
 * Copyright 2020 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rclcpp/rclcpp.hpp>

#include <crs_msgs/srv/crop_toolpaths.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>

#include <tf2_eigen/tf2_eigen.h>

#include <pcl_conversions/pcl_conversions.h>

#include <region_detection_core/region_detector.h>
#include <region_detection_core/region_crop.h>

static const std::string CROP_TOOLPATHS_SERIVCE = "crop_toolpaths";
static const std::string CLOSED_REGIONS_NS = "closed_regions";

typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > EigenPose3dVector;

static geometry_msgs::msg::Pose pose3DtoPoseMsg(const std::array<float, 6>& p)
{
  using namespace Eigen;
  geometry_msgs::msg::Pose pose_msg;
  Eigen::Affine3d eigen_pose = Translation3d(Vector3d(p[0],p[1],p[2])) *
      AngleAxisd(p[3],Vector3d::UnitX()) * AngleAxisd(p[4],Vector3d::UnitY()) *
      AngleAxisd(p[5],Vector3d::UnitZ());

  pose_msg = tf2::toMsg(eigen_pose);
  return std::move(pose_msg);
}

visualization_msgs::msg::MarkerArray convertToAxisMarkers(const std::vector<EigenPose3dVector>& path,
                                                     const std::string& frame_id, const std::string& ns,
                                                     const std::size_t& start_id, const double& axis_scale,
                                                     const double& axis_length,
                                                     const std::array<float, 6>& offset)
{
  using namespace Eigen;

  visualization_msgs::msg::MarkerArray markers;

  auto create_line_marker = [&](const int id, const std::tuple<float, float, float, float>& rgba) -> visualization_msgs::msg::Marker
  {
    visualization_msgs::msg::Marker line_marker;
    line_marker.action = line_marker.ADD;
    std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = rgba;
    line_marker.header.frame_id = frame_id;
    line_marker.type = line_marker.LINE_LIST;
    line_marker.id = id;
    line_marker.lifetime = rclcpp::Duration(0);
    line_marker.ns = ns;
    std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(axis_scale, 0.0, 0.0);
    line_marker.pose = pose3DtoPoseMsg(offset);
    return std::move(line_marker);
  };

  // markers for each axis line
  int marker_id = start_id;
  visualization_msgs::msg::Marker x_axis_marker = create_line_marker(++marker_id,std::make_tuple(1.0, 0.0, 0.0, 1.0));
  visualization_msgs::msg::Marker y_axis_marker = create_line_marker(++marker_id,std::make_tuple(0.0, 1.0, 0.0, 1.0));
  visualization_msgs::msg::Marker z_axis_marker = create_line_marker(++marker_id,std::make_tuple(0.0, 0.0, 1.0, 1.0));

  auto add_axis_line = [](const Isometry3d& eigen_pose, const Vector3d& dir,
      const geometry_msgs::msg::Point& p1, visualization_msgs::msg::Marker& marker){

    geometry_msgs::msg::Point p2;
    Eigen::Vector3d line_endpoint;

    // axis endpoint
    line_endpoint = eigen_pose * dir;
    std::tie(p2.x, p2.y, p2.z)  = std::make_tuple(line_endpoint.x(), line_endpoint.y(), line_endpoint.z());

    // adding line
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  };

  for(auto& poses : path)
  {
    for(auto& pose : poses)
    {
      geometry_msgs::msg::Point p1;
      std::tie(p1.x, p1.y, p1.z) = std::make_tuple(pose.translation().x(), pose.translation().y(),
                                                   pose.translation().z());
      add_axis_line(pose,Vector3d::UnitX()* axis_length,p1, x_axis_marker);
      add_axis_line(pose,Vector3d::UnitY()* axis_length,p1, y_axis_marker);
      add_axis_line(pose,Vector3d::UnitZ()* axis_length,p1, z_axis_marker);
    }
  }

  markers.markers.push_back(x_axis_marker);
  markers.markers.push_back(y_axis_marker);
  markers.markers.push_back(z_axis_marker);
  return std::move(markers);
}

visualization_msgs::msg::MarkerArray convertToDottedLineMarker(const std::vector<EigenPose3dVector>& path,
                                                    const std::string& frame_id,
                                                    const std::string& ns,
                                                    const std::size_t& start_id = 0,
                                                    const std::array<double,4>& rgba_line = {1.0, 1.0, 0.2, 1.0},
                                                    const std::array<double,4>& rgba_point = {0.1, .8, 0.2, 1.0},
                                                    const std::array<float, 6>& offset = {0,0,0,0,0,0},
                                                    const float& line_width = 0.001,
                                                    const float& point_size = 0.0015)
{
  visualization_msgs::msg::MarkerArray markers_msgs;
  visualization_msgs::msg::Marker line_marker, points_marker;

  // configure line marker
  line_marker.action = line_marker.ADD;
  std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = std::make_tuple(rgba_line[0],
                                                                                                                 rgba_line[1],
                                                                                                                 rgba_line[2],
                                                                                                                 rgba_line[3]);
  line_marker.header.frame_id = frame_id;
  line_marker.type = line_marker.LINE_STRIP;
  line_marker.id = start_id;
  line_marker.lifetime = rclcpp::Duration(0);
  line_marker.ns = ns;
  std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(line_width, 0.0, 0.0);
  line_marker.pose = pose3DtoPoseMsg(offset);

  // configure point marker
  points_marker = line_marker;
  points_marker.type = points_marker.POINTS;
  points_marker.ns = ns;
  std::tie(points_marker.color.r, points_marker.color.g, points_marker.color.b, points_marker.color.a) = std::make_tuple(rgba_point[0],
                                                                                                                         rgba_point[1],
                                                                                                                         rgba_point[2],
                                                                                                                         rgba_point[3]);
  std::tie(points_marker.scale.x, points_marker.scale.y, points_marker.scale.z) = std::make_tuple(point_size,point_size,point_size);

  int id_counter = start_id;
  for(auto& poses : path)
  {
    line_marker.points.clear();
    points_marker.points.clear();
    line_marker.points.reserve(poses.size());
    points_marker.points.reserve(poses.size());
    for(auto& pose : poses)
    {
      geometry_msgs::msg::Point p;
      std::tie(p.x, p.y, p.z) = std::make_tuple(pose.translation().x(), pose.translation().y(),
                                                pose.translation().z());
      line_marker.points.push_back(p);
      points_marker.points.push_back(p);
    }

    line_marker.id = (++id_counter);
    points_marker.id = (++id_counter);
    markers_msgs.markers.push_back(line_marker);
    markers_msgs.markers.push_back(points_marker);
  }

  return markers_msgs;
}

class ToolpathCrop
{
public:
	ToolpathCrop(std::shared_ptr<rclcpp::Node> node):
	  node_(node),
	  logger_(node->get_logger())
	{
	  // load parameters

	  // creating service
	  crop_toolpaths_srv_ = node->create_service<crs_msgs::srv::CropToolpaths>(CROP_TOOLPATHS_SERIVCE,
	                                             std::bind(&ToolpathCrop::cropToolpathsCallback,
	                                                       this,
	                                                       std::placeholders::_1,
	                                                       std::placeholders::_2,
	                                                       std::placeholders::_3));

	  static const std::string REGION_MARKERS_TOPIC = "detected_regions";
	  region_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(REGION_MARKERS_TOPIC,rclcpp::QoS(1));

	  // check parameters existences
	  loadRegionDetectionConfig();
	  loadRegionCropConfig();
	}

	~ToolpathCrop()
	{

	}

private:

	region_detection_core::RegionDetectionConfig loadRegionDetectionConfig()
	{
	  region_detection_core::RegionDetectionConfig cfg;

	  // opencv parameters
	  const std::string opencv_param_ns = "config_opencv.";
	  cfg.opencv_cfg.invert_image = node_->get_parameter(opencv_param_ns + "invert_image").as_bool();
	  cfg.opencv_cfg.debug_mode_enable = node_->get_parameter(opencv_param_ns + "debug_mode_enable").as_bool();
	  cfg.opencv_cfg.debug_window_name = node_->get_parameter(opencv_param_ns + "debug_window_name").as_string();
	  cfg.opencv_cfg.debug_wait_key = node_->get_parameter(opencv_param_ns + "debug_wait_key").as_bool();

	  cfg.opencv_cfg.threshold.enable = node_->get_parameter(opencv_param_ns + "threshold.enable").as_bool();
	  cfg.opencv_cfg.threshold.value = node_->get_parameter(opencv_param_ns + "threshold.value").as_int();
	  cfg.opencv_cfg.threshold.type = node_->get_parameter(opencv_param_ns + "threshold.type").as_int();

	  cfg.opencv_cfg.dilation.enable = node_->get_parameter(opencv_param_ns + "dilation.enable").as_bool();
	  cfg.opencv_cfg.dilation.elem = node_->get_parameter(opencv_param_ns + "dilation.elem").as_int();
	  cfg.opencv_cfg.dilation.kernel_size = node_->get_parameter(opencv_param_ns + "dilation.kernel_size").as_int();

	  cfg.opencv_cfg.canny.enable = node_->get_parameter(opencv_param_ns + "canny.enable").as_bool();
	  cfg.opencv_cfg.canny.lower_threshold = node_->get_parameter(opencv_param_ns + "canny.lower_threshold").as_int();
	  cfg.opencv_cfg.canny.upper_threshold = node_->get_parameter(opencv_param_ns + "canny.upper_threshold").as_int();
	  cfg.opencv_cfg.canny.aperture_size = node_->get_parameter(opencv_param_ns + "canny.aperture_size").as_int();

	  cfg.opencv_cfg.contour.mode = node_->get_parameter(opencv_param_ns + "canny.mode").as_int();
	  cfg.opencv_cfg.contour.method = node_->get_parameter(opencv_param_ns + "canny.method").as_int();

	  // pcl 2d parameters
	  const std::string pcl2d_param_ns = "config_pcl2d.";
	  cfg.pcl_2d_cfg.downsampling_radius = node_->get_parameter(pcl2d_param_ns + "downsampling_radius").as_double();
	  cfg.pcl_2d_cfg.split_dist = node_->get_parameter(pcl2d_param_ns + "split_dist").as_double();
	  cfg.pcl_2d_cfg.closed_curve_max_dist = node_->get_parameter(pcl2d_param_ns + "closed_curve_max_dist").as_double();
	  cfg.pcl_2d_cfg.simplification_min_points = node_->get_parameter(pcl2d_param_ns + "simplification_min_points").as_int();
	  cfg.pcl_2d_cfg.simplification_alpha = node_->get_parameter(pcl2d_param_ns + "simplification_alpha").as_double();

	  // pcl 3d parameters
	  const std::string pcl_params_ns = "config_pcl.";
	  cfg.pcl_cfg.debug_mode_enable =  node_->get_parameter(pcl_params_ns + "debug_mode_enable").as_bool();
	  cfg.pcl_cfg.max_merge_dist =  node_->get_parameter(pcl_params_ns + "max_merge_dist").as_double();
	  cfg.pcl_cfg.closed_curve_max_dist =  node_->get_parameter(pcl_params_ns + "closed_curve_max_dist").as_double();
	  cfg.pcl_cfg.simplification_min_dist =  node_->get_parameter(pcl_params_ns + "simplification_min_dist").as_double();
	  cfg.pcl_cfg.min_num_points =  node_->get_parameter(pcl_params_ns + "min_num_points").as_int();

	  cfg.pcl_cfg.stat_removal.enable =  node_->get_parameter(pcl_params_ns + "stat_removal.enable").as_bool();
	  cfg.pcl_cfg.stat_removal.kmeans =  node_->get_parameter(pcl_params_ns + "stat_removal.kmeans").as_int();
	  cfg.pcl_cfg.stat_removal.stddev =  node_->get_parameter(pcl_params_ns + "stat_removal.stddev").as_double();

	  cfg.pcl_cfg.normal_est.kdtree_epsilon=  node_->get_parameter(pcl_params_ns + "normal_est.kdtree_epsilon").as_double();
	  cfg.pcl_cfg.normal_est.search_radius=  node_->get_parameter(pcl_params_ns + "normal_est.search_radius").as_double();
	  std::vector<double> viewpoint=  node_->get_parameter(pcl_params_ns + "normal_est.viewpoint_xyz").as_double_array();
	  cfg.pcl_cfg.normal_est.downsampling_radius=  node_->get_parameter(pcl_params_ns + "normal_est.downsampling_radius").as_double();

	  std::copy(viewpoint.begin(),viewpoint.end(),cfg.pcl_cfg.normal_est.viewpoint_xyz.begin());

	  return cfg;
	}

	region_detection_core::RegionCropConfig loadRegionCropConfig()
	{
	  region_detection_core::RegionCropConfig cfg;
	  const std::string param_ns = "region_crop.";
	  cfg.plane_dist_threshold = node_->get_parameter(param_ns + "plane_dist_threshold").as_double();
	  double heigth_limits_min = node_->get_parameter(param_ns + "heigth_limits_min").as_double();
	  double heigth_limits_max = node_->get_parameter(param_ns + "heigth_limits_max").as_double();
	  cfg.dir_estimation_method = static_cast<region_detection_core::DirectionEstMethods>(
	      node_->get_parameter(param_ns + "plane_dist_threshold").as_int());
	  std::vector<double> user_dir = node_->get_parameter(param_ns + "user_dir").as_double_array();
	  std::vector<double> view_point = node_->get_parameter(param_ns + "view_point").as_double_array();

	  cfg.heigth_limits = std::make_pair(heigth_limits_min,heigth_limits_max);
	  cfg.user_dir = Eigen::Map<Eigen::Vector3d>(user_dir.data(), user_dir.size());
	  cfg.view_point = Eigen::Map<Eigen::Vector3d>(view_point.data(), view_point.size());

	  return cfg;
	}

	void publishRegions(const std::string& frame_id, const std::string ns,const std::vector<EigenPose3dVector>& regions)
	{
    // create markers to publish
    visualization_msgs::msg::MarkerArray region_markers;
    int id = 0;
    for(auto& poses : regions)
    {
      id++;
      visualization_msgs::msg::MarkerArray m = convertToAxisMarkers({poses},frame_id, ns + std::to_string(id),0, 0.001,
                                                               0.01,{0,0,0,0,0,0});
      region_markers.markers.insert(region_markers.markers.end(), m.markers.begin(), m.markers.end());

      m = convertToDottedLineMarker({poses},frame_id,ns  + std::to_string(id));
      region_markers.markers.insert(region_markers.markers.end(), m.markers.begin(), m.markers.end());
    }

    region_markers_pub_->publish(region_markers);
	}

	void cropToolpathsCallback(const std::shared_ptr<rmw_request_id_t> request_header,
	                           const std::shared_ptr<crs_msgs::srv::CropToolpaths::Request> request,
	                           const std::shared_ptr<crs_msgs::srv::CropToolpaths::Response> response)
	{
	  using namespace region_detection_core;
	  using namespace pcl;
	  using Cloud = PointCloud<PointXYZ>;

	  (void)request_header;

	  // converting to input for region detection
	  std::vector<RegionDetector::DataBundle> data_vec;
	  for(std::size_t i = 0; i < request->clouds.size(); i++)
	  {
	    RegionDetector::DataBundle data;
	    pcl_conversions::toPCL(request->clouds[i], data.cloud_blob);
	    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(request->images[i], sensor_msgs::image_encodings::RGBA8);
	    data.image = img->image;
	    data.transform = tf2::transformToEigen(request->transforms[i]);
	    data_vec.push_back(data);
	  }

	  // region detection
	  RegionDetectionConfig config = loadRegionDetectionConfig();
	  RegionDetector region_detector(config);
	  RegionDetector::RegionResults region_detection_results;
	  if(!region_detector.compute(data_vec, region_detection_results))
	  {
      response->succeeded = false;
      response->err_msg = "Failed to find closed regions";
	    RCLCPP_ERROR_STREAM(logger_,response->err_msg );
	    return;
	  }
	  RCLCPP_INFO(logger_,"Found %i closed regions", region_detection_results.closed_regions_poses.size());

	  publishRegions(request->transforms.front().header.frame_id,CLOSED_REGIONS_NS,
	                 region_detection_results.closed_regions_poses);

	  // use detected regions to crop
	  RegionCropConfig region_crop_cfg = loadRegionCropConfig();
	  RegionCrop<pcl::PointXYZ> region_crop;
	  region_crop.setConfig(region_crop_cfg);
	  for(std::size_t  i = 0; i < region_detection_results.closed_regions_poses.size(); i++)
	  {
	    region_crop.setRegion(region_detection_results.closed_regions_poses[i]);

	    for(std::size_t j = 0; j < request->toolpaths.size(); j++)
	    {
	      const crs_msgs::msg::ToolProcessPath& toolpath = request->toolpaths[j];
	      crs_msgs::msg::ToolProcessPath cropped_toolpath;
	      for(std::size_t k = 0; k < toolpath.rasters.size(); k++)
	      {
	        const geometry_msgs::msg::PoseArray& rasters = toolpath.rasters[k];
	        Cloud::Ptr raster_points = boost::make_shared<Cloud>();
	        raster_points->reserve(rasters.poses.size());

	        std::transform(rasters.poses.begin(), rasters.poses.end(), std::back_inserter(*raster_points),[](
	            const geometry_msgs::msg::Pose& pose){
	          PointXYZ p;
	          p.x = pose.position.x;
	          p.y = pose.position.y;
	          p.z = pose.position.z;
	          return p;
	        });

	        region_crop.setInput(raster_points);
	        std::vector<int> inlier_indices = region_crop.filter();

	        if(inlier_indices.empty())
	        {
	          continue;
	        }
	        RCLCPP_INFO(logger_,"Found %i inliers in of toolpath %j region %i", inlier_indices.size(), j, i);

	        // extracting poinst in raster
	        geometry_msgs::msg::PoseArray cropped_raster;
	        std::for_each(inlier_indices.begin(), inlier_indices.end(),[&cropped_raster,&rasters](int idx){
	          cropped_raster.poses.push_back(rasters.poses[idx]);
	        });
	        cropped_toolpath.rasters.push_back(cropped_raster);
	      }
	      response->cropped_toolpaths.push_back(cropped_toolpath);
	    }
	  }

	  if(response->cropped_toolpaths.empty())
	  {
      response->succeeded = false;
      response->err_msg = "Failed to crop toolpaths";
	    RCLCPP_ERROR_STREAM(logger_,response->err_msg);
	  }

	  response->succeeded = true;
	}

	rclcpp::Service<crs_msgs::srv::CropToolpaths>::SharedPtr crop_toolpaths_srv_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr region_markers_pub_;
	std::shared_ptr<rclcpp::Node> node_;
	rclcpp::Logger logger_;

};

int main(int argc, char** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides();
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("toolpath_crop",options);
  ToolpathCrop toolpath_crop(node);
  rclcpp::spin(node);
  return 0;
}

