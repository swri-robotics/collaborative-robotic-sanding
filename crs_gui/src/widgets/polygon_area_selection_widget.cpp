/*
 * Copyright 2019 Southwest Research Institute
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

#include "crs_gui/widgets/polygon_area_selection_widget.h"

#include <QMessageBox>
#include <QPushButton>

#include <std_srvs/srv/trigger.h>
#include <pcl/io/ply_io.h>

#include <ui_polygon_area_selection_widget.h>

namespace
{
/**
 * @brief ConvertToMeshMsg Converts a polygon mesh to a mesh msgs
 *
 * TODO: Port all/most of noether conversions to ROS 2 and pull it into crs
 * @param mesh Input pcl::PolygonMesh
 * @param mesh_msg Resulting shape_msgs::Mesh
 * @return true if successful
 */
bool ConvertToMeshMsg(const pcl::PolygonMesh& mesh, shape_msgs::msg::Mesh& mesh_msg)
{
  if (mesh.polygons.empty())
  {
    std::cerr << "PolygonMesh has no polygons" << std::endl;
    //    CONSOLE_BRIDGE_logError("PolygonMesh has no polygons");
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  if (cloud.empty())
  {
    //    CONSOLE_BRIDGE_logError("PolygonMesh has no vertices data");
    return false;
  }

  // copying triangles
  mesh_msg.triangles.resize(mesh.polygons.size());
  for (std::size_t i = 0; i < mesh.polygons.size(); i++)
  {
    const pcl::Vertices& vertices = mesh.polygons[i];
    if (vertices.vertices.size() != 3)
    {
      //      CONSOLE_BRIDGE_logError("Vertex in PolygonMesh needs to have 3 elements only");
      return false;
    }

    std::array<uint32_t, 3>& vertex = mesh_msg.triangles[i].vertex_indices;
    std::tie(vertex[0], vertex[1], vertex[2]) =
        std::make_tuple(vertices.vertices[0], vertices.vertices[1], vertices.vertices[2]);
  }

  // copying vertices
  mesh_msg.vertices.resize(cloud.size());
  std::transform(cloud.begin(), cloud.end(), mesh_msg.vertices.begin(), [](pcl::PointXYZ& v) {
    geometry_msgs::msg::Point p;
    std::tie(p.x, p.y, p.z) = std::make_tuple(v.x, v.y, v.z);
    return std::move(p);
  });
  return true;
}
}  // namespace

namespace crs_gui
{
PolygonAreaSelectionWidget::PolygonAreaSelectionWidget(rclcpp::Node::SharedPtr node,
                                                       const std::string& selection_world_frame,
                                                       const std::string& selection_sensor_frame,
                                                       QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::PolygonAreaSelectionWidget)
  , node_(node)
  , selector_(node, selection_world_frame, selection_sensor_frame)
{
  ui_->setupUi(this);
  connect(
      ui_->push_button_clear_selection, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::clearROISelection);
  connect(ui_->push_button_apply_selection, &QPushButton::clicked, this, &PolygonAreaSelectionWidget::applySelection);
}

PolygonAreaSelectionWidget::~PolygonAreaSelectionWidget() = default;

void PolygonAreaSelectionWidget::loadShapeMsg(const shape_msgs::msg::Mesh& mesh)
{
  mesh_.reset(new shape_msgs::msg::Mesh(mesh));
  clearROISelection();
  return;
}

void PolygonAreaSelectionWidget::loadMeshFile(const std::string& filepath)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Area selection loading file: " << filepath.c_str());
  pcl::PolygonMesh polygon_mesh;
  if (pcl::io::loadPLYFile(filepath, polygon_mesh))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Mesh loading failed");
    return;
  }

  shape_msgs::msg::Mesh shape_msg;
  if (ConvertToMeshMsg(polygon_mesh, shape_msg))
  {
    mesh_ = std::make_shared<shape_msgs::msg::Mesh>(shape_msg);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Mesh loaded");
  }
  else
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Mesh failed to convert");
  }
}

void PolygonAreaSelectionWidget::clearROISelection()
{
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  // Currently, this callback is being called directly, so it will always return true.
  selector_.clearROIPointsCb(req, res);
  if (!res->success)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Tool Path Parameter Editor Widget: Area Selection error:" << res->message);
  }
  // This will be uncommented when mesh selection is turned back on
  //  submesh_.reset(new shape_msgs::msg::Mesh(*mesh_));

  //  emit(selectedSubmesh(submesh_));
  return;
}

void PolygonAreaSelectionWidget::applySelection()
{
  if (!mesh_)
  {
    // TODO: Currently this will always happen because we are not setting a mesh. Basically we are just drawing on the
    // surface an not doing anything
    QMessageBox::warning(this, "Tool Path Planning Error", "No mesh available to crop");
    return;
  }
  submesh_.reset(new shape_msgs::msg::Mesh());
  std::string error_message;
  bool success = selector_.collectROIMesh(*mesh_, *submesh_, error_message);
  if (!success)
  {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Tool Path Parameter Editor Widget: Area Selection error: could not compute submesh: " << error_message);
  }
  if (submesh_->vertices.size() < 3 || submesh_->triangles.size() < 1)
  {
    submesh_.reset(new shape_msgs::msg::Mesh(*mesh_));
  }

  emit(selectedSubmesh(submesh_));
  return;
}

}  // namespace crs_gui
