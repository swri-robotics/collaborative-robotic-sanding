/*
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

#ifndef CRS_GUI_REGISTER_ROS_MSGS_FOR_QT_H
#define CRS_GUI_REGISTER_ROS_MSGS_FOR_QT_H

#include <QMetaType>

#include <shape_msgs/msg/mesh.hpp>

Q_DECLARE_METATYPE(shape_msgs::msg::Mesh);
Q_DECLARE_METATYPE(shape_msgs::msg::Mesh::SharedPtr);
Q_DECLARE_METATYPE(std::vector<shape_msgs::msg::Mesh>);
Q_DECLARE_METATYPE(std::vector<shape_msgs::msg::Mesh::SharedPtr>);

#endif  // OPP_GUI_REGISTER_ROS_MSGS_FOR_QT_H
