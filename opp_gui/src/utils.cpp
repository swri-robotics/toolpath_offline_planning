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

#include "opp_gui/utils.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>


namespace opp_gui
{
namespace utils
{

namespace vm = visualization_msgs;

bool getMeshMsgFromResource(const std::string& resource,
                            shape_msgs::Mesh& mesh_msg)
{
  shapes::Mesh* mesh = shapes::createMeshFromResource(resource);
  if(!mesh)
  {
    ROS_ERROR_STREAM("Failed to load mesh from resource: '" << resource << "'");
    return false;
  }

  shapes::ShapeMsg shape_msg;
  if(!shapes::constructMsgFromShape(mesh, shape_msg))
  {
    ROS_ERROR_STREAM(__func__ << ": Failed to create shape message from mesh");
    return false;
  }

  mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

  return true;
}

std_msgs::ColorRGBA createColor(const float r,
                                const float g,
                                const float b,
                                const float a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

vm::Marker createArrowMarker(const int id,
                             const std::string& ns,
                             const Eigen::Isometry3d& pose,
                             const std::string& frame,
                             const std_msgs::ColorRGBA& color,
                             const vm::Marker::_action_type& action)
{
  vm::Marker marker;

  marker.header.frame_id = frame;
  marker.id = id;
  marker.ns = ns;
  marker.action = action;
  marker.color = color;

  marker.type = vm::Marker::ARROW;
  double length = 0.25;

  geometry_msgs::Point head;
  tf::pointEigenToMsg(pose.matrix().col(3).head<3>(), head);

  geometry_msgs::Point tail;
  tf::pointEigenToMsg(pose * Eigen::Vector3d(0.0, 0.0, -length), tail);

  marker.points.reserve(2);
  marker.points.push_back(tail);
  marker.points.push_back(head);

  marker.scale.x = length / 10.0;
  marker.scale.y = length * 2.0 / 10.0;
  marker.scale.z = length * 2.0 / 10.0;

  return marker;
}

vm::Marker createMeshMarker(const int id,
                            const std::string& ns,
                            const Eigen::Isometry3d& pose,
                            const std::string& frame,
                            const std::string& mesh_resource,
                            const std_msgs::ColorRGBA& color,
                            const vm::Marker::_action_type& action)
{
  vm::Marker marker;

  marker.header.frame_id = frame;
  marker.id = id;
  marker.ns = ns;
  marker.action = action;

  marker.type = vm::Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_resource;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color = color;

  tf::poseEigenToMsg(pose, marker.pose);

  return marker;
}

QStringList toQStringList(const std::vector<std::string>& in)
{
  QStringList out;
  out.reserve(in.size());
  for(const std::string& str : in)
  {
    out.push_back(QString::fromStdString(str));
  }
  return out;
}


bool pclMsgToShapeMsg(const pcl_msgs::PolygonMesh& pcl_mesh_msg, shape_msgs::Mesh& mesh_msg)
{
  // Convert msg to mesh
  pcl::PolygonMesh pcl_mesh;
  pcl_conversions::toPCL(pcl_mesh_msg, pcl_mesh);

  // Make sure that there are at least three points and at least one polygon
  if (pcl_mesh.cloud.height * pcl_mesh.cloud.width < 3 || pcl_mesh.polygons.size() < 1)
  {
    return false;
  }

  // Prepare the message's vectors to receive data
  // One resize now saves time later
  mesh_msg.vertices.resize(pcl_mesh.cloud.height * pcl_mesh.cloud.width);
  mesh_msg.triangles.resize(pcl_mesh.polygons.size());

  // Get the points from the pcl mesh
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(pcl_mesh.cloud, vertices);

  // Copy the coordinates inside the vertices into the new mesh
  // TODO: Maybe check for nan?
  for (std::size_t i = 0; i < vertices.size(); ++i)
  {
    mesh_msg.vertices[i].x = static_cast<double>(vertices[i]._PointXYZ::x);
    mesh_msg.vertices[i].y = static_cast<double>(vertices[i]._PointXYZ::y);
    mesh_msg.vertices[i].z = static_cast<double>(vertices[i]._PointXYZ::z);
  }

  // Copy the vertex indices (which describe each polygon) from
  // the old mesh to the new mesh
  for (std::size_t i = 0; i < pcl_mesh.polygons.size(); ++i)
  {
    // If a 'polygon' in the old mesh did not have 3 or more vertices,
    // throw an error.  (If the polygon has 4 or more, just use the first
    // 3 to make a triangle.)
    // TODO: It is possible to decompose any polygon into multiple
    // triangles, and that would prevent loss of data here.
    if (pcl_mesh.polygons[i].vertices.size() < 3)
    {
      return false;
    }
    mesh_msg.triangles[i].vertex_indices[0] = pcl_mesh.polygons[i].vertices[0];
    mesh_msg.triangles[i].vertex_indices[1] = pcl_mesh.polygons[i].vertices[1];
    mesh_msg.triangles[i].vertex_indices[2] = pcl_mesh.polygons[i].vertices[2];
  }

  return true;
}

bool pclMsgFromShapeMsg(const shape_msgs::Mesh& mesh_msg, pcl_msgs::PolygonMesh& pcl_mesh_msg)
{
  pcl::PolygonMesh pcl_mesh;

  // Make sure that there are at least three points and at least one polygon
  if (mesh_msg.vertices.size() < 3 || mesh_msg.triangles.size() < 1)
  {
    return false;
  }

  // Prepare PCL structures to receive data
  // Resizing once now saves time later
  pcl::PointCloud<pcl::PointXYZ> vertices;
  vertices.resize(mesh_msg.vertices.size());
  pcl_mesh.polygons.resize(mesh_msg.triangles.size());

  // Copy the vertex indices (which describe each polygon) from
  // the old mesh to the new mesh
  for (std::size_t i = 0; i < mesh_msg.vertices.size(); ++i)
  {
    vertices[i]._PointXYZ::x = static_cast<float>(mesh_msg.vertices[i].x);
    vertices[i]._PointXYZ::y = static_cast<float>(mesh_msg.vertices[i].y);
    vertices[i]._PointXYZ::z = static_cast<float>(mesh_msg.vertices[i].z);
  }

  // Copy the vertex indices (which describe each polygon) from
  // the old mesh to the new mesh
  for (std::size_t i = 0; i < mesh_msg.triangles.size(); ++i)
  {
    pcl_mesh.polygons[i].vertices.resize(3);
    pcl_mesh.polygons[i].vertices[0] = mesh_msg.triangles[i].vertex_indices[0];
    pcl_mesh.polygons[i].vertices[1] = mesh_msg.triangles[i].vertex_indices[1];
    pcl_mesh.polygons[i].vertices[2] = mesh_msg.triangles[i].vertex_indices[2];
  }

  // Use the filled pointcloud to populate the pcl mesh
  pcl::toPCLPointCloud2(vertices, pcl_mesh.cloud);

  // Convert to message
  pcl_conversions::fromPCL(pcl_mesh, pcl_mesh_msg);

  return true;
}

} // namespace utils
} // namespace opp_gui
