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

#ifndef OPP_GUI_UTILS_H
#define OPP_GUI_UTILS_H

#include <QStringList>

#include <Eigen/Geometry>
#include <pcl_msgs/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <visualization_msgs/Marker.h>

namespace opp_gui
{
namespace utils
{
/**
 * @brief Method for loading a mesh from a resource file location
 * @param resource: the file location of the mesh in the format `package://<package_name>/.../<filename>.stl` or
 * `file:///<path>/<to>/<filename>.stl`
 * @param mesh_msg: the loaded mesh
 * @return
 */
bool getMeshMsgFromResource(const std::string& resource, shape_msgs::Mesh& mesh_msg);

std_msgs::ColorRGBA createColor(const float r, const float g, const float b, const float a = 1.0f);

visualization_msgs::Marker
createArrowMarker(const int id,
                  const std::string& ns,
                  const Eigen::Isometry3d& pose,
                  const std::string& frame,
                  const std_msgs::ColorRGBA& color = createColor(0.0f, 1.0f, 0.0f),
                  const visualization_msgs::Marker::_action_type& action = visualization_msgs::Marker::ADD);

visualization_msgs::Marker
createMeshMarker(const int id,
                 const std::string& ns,
                 const Eigen::Isometry3d& pose,
                 const std::string& frame,
                 const std::string& mesh_resource,
                 const std_msgs::ColorRGBA& color = createColor(0.0f, 0.75f, 0.0f, 0.25f),
                 const visualization_msgs::Marker::_action_type& action = visualization_msgs::Marker::ADD);

QStringList toQStringList(const std::vector<std::string>& in);

bool pclMsgToShapeMsg(const pcl_msgs::PolygonMesh& pcl_mesh, shape_msgs::Mesh& mesh_msg);

bool pclMsgFromShapeMsg(const shape_msgs::Mesh& mesh_msg, pcl_msgs::PolygonMesh& pcl_mesh);

}  // namespace utils
}  // namespace opp_gui

#endif  // OPP_GUI_UTILS_H
