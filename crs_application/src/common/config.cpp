/*
 * @author Jorge Nicho
 * @file config.cpp
 * @date Feb 25, 2020
 * @copyright Copyright (c) 2020, Southwest Research Institute
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/format.hpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/emitter.h>
#include <crs_application/common/config.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

static rclcpp::Logger CONFIG_LOGGER = rclcpp::get_logger("CRS_CONFIG");

namespace config_fields
{
  namespace motion_planning
  {
    static const std::string TOP_LEVEL = "motion_planning";
    static const std::string HOME_POS_ROOT= "home_position";
    static const std::string PROCESS_PATH_ROOT= "process_path";
    static const std::string MEDIA_CHANGE_ROOT= "media_change";
    static const std::string PREVIEW_ROOT= "preview";

    static const std::vector<std::string> HOME_POS_ITEMS = {"joint_names", "joint_position"};
    static const std::vector<std::string> PROCESS_PATH_ITEMS = {"tool_speed", "offset_pose",
                                                          "retreat_dist", "approach_dist","tool_frame"};
    static const std::vector<std::string> MEDIA_CHANGE_ITEMS = {"change_time", "change_pose"};
    static const std::vector<std::string> PREVIEW_ITEMS = {"time_scaling"};
  }

  namespace process_execution
  {
    static const std::string TOP_LEVEL = "process_execution";
    static const std::string TIME_TOLERANCE= "time_tolerance";
    static const std::string JOINT_TOLERANCE= "joint_tolerance"; // vector<double>
  }

  namespace scan_acquistion
  {
    static const std::string TOP_LEVEL = "scan_acquisition";
    static const std::string SCAN_POSES_ROOT= "scan_poses";
    static const std::vector<std::string> SCAN_POSES_ITEMS = {"pose"};
    static const std::string TOOL_FRAME = "tool_frame";
    static const std::string SKIP_ON_FAILURE = "skip_on_failure";
  }

  namespace part_registration
  {
    static const std::string TOP_LEVEL = "part_registration";
    static const std::string TARGET_FRAME_ID = "target_frame_id";
    static const std::string PART_FILE = "part_file";
    static const std::string TOOLPATH_FILE = "toolpath_file";
  }
}

static bool hasFields(YAML::Node& n, const std::string& parent_name, const std::vector<std::string>& fields)
{
  for(const auto& f : fields)
  {
    if(!n[f])
    {
      RCLCPP_ERROR(CONFIG_LOGGER,"Field '%s' was not found in '%s' element ", f.c_str(), parent_name.c_str());
      return false;
    }
  }
  return true;
}

namespace crs_application
{
namespace config
{
  template<>
  boost::optional<MotionPlanningConfig> parse(YAML::Node& config, std::string& err_msg)
  {
    using namespace YAML;
    using namespace config_fields::motion_planning;
    MotionPlanningConfig cfg;
    try
    {
      Node root_node = config[TOP_LEVEL];
      if(!root_node)
      {
        err_msg = boost::str(boost::format("The '%s' field was not found") % TOP_LEVEL);
        return boost::none;
      }

      Node home_pose_node = root_node[HOME_POS_ROOT];
      if(home_pose_node && hasFields(home_pose_node, HOME_POS_ROOT, HOME_POS_ITEMS))
      {
        cfg.joint_names = home_pose_node[HOME_POS_ITEMS[0]].as<std::vector<std::string>>();
        cfg.joint_home_position = home_pose_node[HOME_POS_ITEMS[1]].as<std::vector<double>>();
      }
      else
      {
        return boost::none;
      }

      Node process_path_node = root_node[PROCESS_PATH_ROOT];
      if(process_path_node && hasFields(process_path_node, PROCESS_PATH_ROOT, PROCESS_PATH_ITEMS))
      {
        cfg.tool_speed = process_path_node[PROCESS_PATH_ITEMS[0]].as<double>();
        std::vector<double> xyzrpy = process_path_node[PROCESS_PATH_ITEMS[1]].as<std::vector<double>>();
        cfg.retreat_dist = process_path_node[PROCESS_PATH_ITEMS[2]].as<double>();
        cfg.approac_dist = process_path_node[PROCESS_PATH_ITEMS[3]].as<double>();
        cfg.tool_frame = process_path_node[PROCESS_PATH_ITEMS[4]].as<std::string>();

        cfg.offset_pose = Eigen::Translation3d(Eigen::Vector3d(xyzrpy[0],xyzrpy[1], xyzrpy[2])) *
            Eigen::AngleAxisd(xyzrpy[3],Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(xyzrpy[4],Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(xyzrpy[5],Eigen::Vector3d::UnitZ());
      }
      else
      {
        return boost::none;
      }

      Node media_change_node = root_node[MEDIA_CHANGE_ROOT];
      if(media_change_node && hasFields(media_change_node, MEDIA_CHANGE_ROOT, MEDIA_CHANGE_ITEMS))
      {
        cfg.media_change_time = media_change_node[MEDIA_CHANGE_ITEMS[0]].as<double>();
        std::vector<double> xyzrpy = media_change_node[MEDIA_CHANGE_ITEMS[1]].as<std::vector<double>>();
        cfg.media_change_pose = Eigen::Translation3d(Eigen::Vector3d(xyzrpy[0],xyzrpy[1], xyzrpy[2])) *
            Eigen::AngleAxisd(xyzrpy[3],Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(xyzrpy[4],Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(xyzrpy[5],Eigen::Vector3d::UnitZ());
      }
      else
      {
        return boost::none;
      }

      Node preview_node = root_node[PREVIEW_ROOT];
      if(preview_node && hasFields(preview_node, PREVIEW_ROOT, PREVIEW_ITEMS))
      {
        cfg.preview_time_scaling = preview_node[PREVIEW_ITEMS[0]].as<double>();
      }
      else
      {
        return boost::none;
      }
    }
    catch(InvalidNode& e)
    {
      err_msg = boost::str(boost::format("Failed to parse yaml: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(BadConversion& e)
    {
      err_msg = boost::str(boost::format("Failed to parse yaml: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(KeyNotFound& e)
    {
      err_msg = boost::str(boost::format("Key not found: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }

    return cfg;
  }

  template<>
  boost::optional<ProcessExecutionConfig> parse(YAML::Node& config, std::string& err_msg)
  {
    using namespace YAML;
    using namespace config_fields::process_execution;
    ProcessExecutionConfig cfg;

    try
    {
      Node root_node = config[TOP_LEVEL];
      if(root_node && hasFields(root_node, TOP_LEVEL, {TIME_TOLERANCE, JOINT_TOLERANCE}))
      {
        cfg.traj_time_tolerance = root_node[TIME_TOLERANCE].as<double>();
        cfg.joint_tolerance = root_node[JOINT_TOLERANCE].as<double>();
      }
      else
      {
        return boost::none;
      }
    }
    catch(InvalidNode& e)
    {
      err_msg = boost::str(boost::format("Failed to parse yaml: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(BadConversion& e)
    {
      err_msg = boost::str(boost::format("Failed to parse yaml: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(KeyNotFound& e)
    {
      err_msg = boost::str(boost::format("Key not found: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    return cfg;
  }

  template<>
  boost::optional<ScanAcquisitionConfig> parse(YAML::Node& config, std::string& err_msg)
  {
    using namespace YAML;
    using namespace config_fields::scan_acquistion;
    ScanAcquisitionConfig cfg;
    try
    {


      Node root_node = config[TOP_LEVEL];
      if(!root_node)
      {
        err_msg = boost::str(boost::format("The '%s' field was not found") % TOP_LEVEL);
        return boost::none;
      }

      if(hasFields(root_node,TOP_LEVEL, {SCAN_POSES_ROOT, TOOL_FRAME, SKIP_ON_FAILURE}))
      {
        cfg.tool_frame = root_node[TOOL_FRAME].as<std::string>();
        cfg.skip_on_failure = root_node[SKIP_ON_FAILURE].as<bool>();
      }
      else
      {
        return boost::none;
      }

      Node scan_poses_node = root_node[SCAN_POSES_ROOT];
      if(scan_poses_node.Type() == NodeType::Sequence)
      {
        for(std::size_t i = 0; i < scan_poses_node.size(); i++)
        {
          Node item_node = scan_poses_node[i];
          if(!hasFields(item_node,SCAN_POSES_ROOT, SCAN_POSES_ITEMS))
          {
            return boost::none;
          }
          std::vector<double> pose_vals = item_node[SCAN_POSES_ITEMS[0]].as<std::vector<double> >();
          cfg.scan_poses.push_back(pose_vals);
        }
      }
      else
      {
        return boost::none;
      }
    }
    catch(InvalidNode& e)
    {
      err_msg = boost::str(boost::format("Invalid node: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(BadConversion& e)
    {
      err_msg = boost::str(boost::format("Failed to parse yaml: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(KeyNotFound& e)
    {
      err_msg = boost::str(boost::format("Key not found: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    return cfg;
  }

  template<>
  boost::optional<PartRegistrationConfig> parse(YAML::Node& config, std::string& err_msg)
  {
    using namespace YAML;
    using namespace config_fields::part_registration;
    PartRegistrationConfig cfg;
    try
    {
      Node root_node = config[TOP_LEVEL];
      if(!root_node)
      {
        err_msg = boost::str(boost::format("The '%s' field was not found") % TOP_LEVEL);
        return boost::none;
      }

      if(hasFields(root_node,TOP_LEVEL, {TARGET_FRAME_ID, PART_FILE, TOOLPATH_FILE}))
      {
        cfg.target_frame_id = root_node[TARGET_FRAME_ID].as<std::string>();
        cfg.part_file = root_node[PART_FILE].as<std::string>();
        cfg.toolpath_file = root_node[TOOLPATH_FILE].as<bool>();
      }
      else
      {
        return boost::none;
      }
    }
    catch(InvalidNode& e)
    {
      err_msg = boost::str(boost::format("Failed to parse yaml: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(BadConversion& e)
    {
      err_msg = boost::str(boost::format("Failed to parse yaml: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    catch(KeyNotFound& e)
    {
      err_msg = boost::str(boost::format("Key not found: %s") % e.what());
      RCLCPP_ERROR(CONFIG_LOGGER,"%s",err_msg.c_str());
      return boost::none;
    }
    return cfg;
  }
}
}
