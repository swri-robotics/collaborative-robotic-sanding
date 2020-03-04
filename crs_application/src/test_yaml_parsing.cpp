/*
 * test_yaml_parsing.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: jnicho
 */

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/emitter.h>
#include <boost/filesystem.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

static const std::string CONFIG_ROOT_ITEM = "crs";
static const std::vector<std::string> CONFIG_REQUIRED_FIELDS = {"general", "motion_planning","scan_acquisition",
                                                   "process_execution", "part_registration"};
static const std::vector<std::string> CONFIG_PART_REG_FIELDS = {"part_file", "toolpath_file"};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("TEST YAML");
int main(int argc, char** argv)
{
  namespace fs = boost::filesystem;

  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);


  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("test_yaml");
  rclcpp::spin_some(node);
  if(argc < 2)
  {
    std::cout<<"Got less than 2 args"<<std::endl;
    RCLCPP_ERROR(LOGGER,"Got less than 2 args");
    return -1;
  }

  std::string yaml_file = argv[1];
  if(!fs::exists(fs::path(yaml_file)))
  {
    RCLCPP_ERROR(LOGGER, "File %s does not exists", yaml_file.c_str());
    return -1;
  }

  std::cout<<"Loading file "<<yaml_file <<std::endl;
  YAML::Node config_node_ = YAML::LoadFile(yaml_file);
  if(!config_node_)
  {
    RCLCPP_ERROR(LOGGER,"Failed to load config file %s", yaml_file.c_str());
    return false;
  }

  // check fields
  try
  {
    if(!config_node_[CONFIG_ROOT_ITEM])
    {
      RCLCPP_ERROR(LOGGER, "Top element %s was not found in the yaml", CONFIG_ROOT_ITEM.c_str());
      return false;
    }

    YAML::Node crs_node = config_node_[CONFIG_ROOT_ITEM];
    if(!crs_node || std::any_of(CONFIG_REQUIRED_FIELDS.begin(), CONFIG_REQUIRED_FIELDS.end(), [&crs_node] (
        const std::string& f){
      auto node = crs_node[f];
      if(!node)
      {
        RCLCPP_ERROR(LOGGER,"Config file %s is missing required field '%s'", f.c_str());
        return true;
      }
      std::cout<<"Found node :"<< f <<std::endl;
      std::cout<<"\t"<< node <<std::endl;
      return false;
    }))
    {
      return false;
    }
  }
  catch(YAML::InvalidNode& e)
  {
    RCLCPP_ERROR(LOGGER,"Failed to parse config file %s with msg: %S", yaml_file.c_str(), e.what());
  }

  std::cout<<"YAML file passed all checks" <<std::endl;

  rclcpp::spin_some(node);

  return 0;
}


