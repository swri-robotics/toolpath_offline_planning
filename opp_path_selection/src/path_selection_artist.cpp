/*
 * Copyright 2018 Southwest Research Institute
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

#include "opp_path_selection/path_selection_artist.h"

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <message_serialization/serialize.h>
#include <ros/package.h>
#include <shape_msgs/Mesh.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "opp_path_selection/path_selector.h"
#include <opp_msgs/GetPathSelectionMesh.h>

namespace opp_path_selection
{
const static double TF_LOOKUP_TIMEOUT = 5.0;
const static std::string MARKER_ARRAY_TOPIC = "path_markers";
const static std::string CLICKED_POINT_TOPIC = "clicked_point";
const static std::string CLEAR_PATH_POINTS_SERVICE = "clear_path_selection_points";
const static std::string COLLECT_PATH_POINTS_SERVICE = "collect_path_selection_points";
const static std::string path_selection_config_file = ros::package::getPath("opp_path_selection") + "/config/"
                                                                                                    "path_selection_"
                                                                                                    "parameters.yaml";

}  // namespace opp_path_selection

namespace
{
std::vector<visualization_msgs::Marker> makeVisual(const std::string& frame_id)
{
  visualization_msgs::Marker points, lines;
  points.header.frame_id = lines.header.frame_id = frame_id;
  points.ns = lines.ns = "path_selection";
  points.action = lines.action = visualization_msgs::Marker::ADD;

  // Point specific properties
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.scale.x = points.scale.y = points.scale.z = 0.025;
  points.color.b = points.color.a = 1.0;
  points.pose.orientation.w = 1.0;

  // Line specific properties
  lines.id = 1;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.scale.x = 0.01;
  lines.color.b = lines.color.a = 1.0;
  lines.pose.orientation.w = 1.0;

  std::vector<visualization_msgs::Marker> visuals;
  visuals.push_back(points);
  visuals.push_back(lines);
  return visuals;
}

bool pclToShapeMsg(const pcl::PolygonMesh& pcl_mesh, shape_msgs::Mesh& mesh_msg)
{
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

bool pclFromShapeMsg(const shape_msgs::Mesh& mesh_msg, pcl::PolygonMesh& pcl_mesh)
{
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

  return true;
}

}  // namespace

namespace opp_path_selection
{
PathSelectionArtist::PathSelectionArtist(const ros::NodeHandle& nh,
                                         const std::string& world_frame,
                                         const std::string& sensor_frame)
  : nh_(nh), world_frame_(world_frame), sensor_frame_(sensor_frame)
{
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(MARKER_ARRAY_TOPIC, 1, true);
  listener_.reset(new tf::TransformListener(nh_));

  if (!listener_->waitForTransform(sensor_frame_, world_frame_, ros::Time(0), ros::Duration(TF_LOOKUP_TIMEOUT)))
  {
    ROS_ERROR("Transform lookup from %s to %s timed out", sensor_frame_.c_str(), world_frame_.c_str());
    throw std::runtime_error("Transform lookup timed out");
  }

  clear_path_points_srv_ =
      nh_.advertiseService(CLEAR_PATH_POINTS_SERVICE, &PathSelectionArtist::clearPathPointsCb, this);
  collect_path_points_srv_ =
      nh_.advertiseService(COLLECT_PATH_POINTS_SERVICE, &PathSelectionArtist::collectPathPointsMeshCb, this);

  // Initialize subscribers and callbacks
  boost::function<void(const geometry_msgs::PointStampedConstPtr&)> drawn_points_cb;
  drawn_points_cb = boost::bind(&PathSelectionArtist::addSelectionPoint, this, _1);
  drawn_points_sub_ = nh_.subscribe(CLICKED_POINT_TOPIC, 1, drawn_points_cb);

  marker_array_.markers = makeVisual(sensor_frame);

  enabled_ = true;
}

bool PathSelectionArtist::clearPathPointsCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  for (auto it = marker_array_.markers.begin(); it != marker_array_.markers.end(); ++it)
  {
    it->points.clear();
  }

  marker_pub_.publish(marker_array_);
  res.success = true;
  res.message = "Selection cleared";

  return true;
}

bool PathSelectionArtist::collectPath(const shape_msgs::Mesh& mesh_msg,
                                      std::vector<int>& points_idx,
                                      std::string& message)
{
  // first get the mesh
  pcl::PolygonMesh mesh;
  pclFromShapeMsg(mesh_msg, mesh);
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
  opp_msgs::GetPathSelectionCloud srv;
  pcl::toROSMsg(mesh_cloud, srv.request.input_cloud);

  // now get the path segments from markers and find point indices from mesh along each segment
  bool success = collectPathPointsCloudCb(srv.request, srv.response);
  message = srv.response.message;
  points_idx.clear();
  if (!success)
  {
    ROS_ERROR("collectPathPointsCloudCb failed");
    return false;
  }
  if (!srv.response.success)  // no points found
  {
    points_idx.clear();  // this is not a failure, we allow zero points for the heat method
    return true;
  }

  // the response includes res.cloud_indices which represent the path of vertices between each segment of markers/points
  for (int i = 0; i < srv.response.cloud_indices.size(); i++)
  {
    int idx = srv.response.cloud_indices[i];
    points_idx.push_back(idx);
  }
  return true;
}

bool PathSelectionArtist::collectPathMesh(const shape_msgs::Mesh& mesh_msg,
                                          std::vector<int>& points_idx,
                                          std::string& message)
{
  // first get the mesh
  opp_msgs::GetPathSelectionMesh srv;
  srv.request.input_mesh = mesh_msg;

  // now get the path segments from markers and find point indices from mesh along each segment
  bool success = collectPathPointsMeshCb(srv.request, srv.response);
  message = srv.response.message;
  points_idx.clear();
  if (!success)
  {
    return false;
  }
  if (!srv.response.success)
  {
    message = "No points found on mesh";
  }

  // the response includes res.cloud_indices which represent the path of vertices between each segment of markers/points
  for (int i = 0; i < srv.response.mesh_indices.size(); i++)
  {
    int idx = srv.response.mesh_indices[i];
    points_idx.push_back(idx);
  }

  return true;
}

bool PathSelectionArtist::collectPathPointsCloudCb(opp_msgs::GetPathSelectionCloudRequest& req,
                                                   opp_msgs::GetPathSelectionCloudResponse& res)
{
  // get points from existing marker_array_
  auto points_it = std::find_if(marker_array_.markers.begin(),
                                marker_array_.markers.end(),
                                [](const visualization_msgs::Marker& marker) { return marker.id == 0; });

  // Convert the selection points to Eigen vectors
  std::vector<Eigen::Vector3d> points;
  for (const geometry_msgs::Point& pt : points_it->points)
  {
    Eigen::Vector3d e;
    tf::pointMsgToEigen(pt, e);
    points.push_back(std::move(e));
  }

  if (points.size() < 2)
  {
    res.cloud_indices.clear();
    res.success = false;
    res.message = "No Points Selected, use Geometric Axis";
    return true;
  }

  // get point cloud from service request
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(req.input_cloud, *cloud);

  // get all indices from provided mesh vertices along path
  // TODO replace getRegionOfInterest with a path equivalent
  PathSelector sel;
  res.cloud_indices = sel.findPointsAlongSegments(cloud, points);

  if (!res.cloud_indices.empty())
  {
    res.success = true;
    res.message = "Selection complete";
  }
  else
  {
    res.success = false;
    res.message = "Unable to identify points along selected path";
  }

  return true;
}

bool PathSelectionArtist::collectPathPointsMeshCb(opp_msgs::GetPathSelectionMeshRequest& req,
                                                  opp_msgs::GetPathSelectionMeshResponse& res)
{
  // get points from existing marker_array_
  auto points_it = std::find_if(marker_array_.markers.begin(),
                                marker_array_.markers.end(),
                                [](const visualization_msgs::Marker& marker) { return marker.id == 0; });

  // Convert the selection points to Eigen vectors
  std::vector<Eigen::Vector3d> points;
  for (const geometry_msgs::Point& pt : points_it->points)
  {
    Eigen::Vector3d e;
    tf::pointMsgToEigen(pt, e);
    points.push_back(std::move(e));
  }

  if (points.size() < 2)
  {
    res.mesh_indices.clear();
    res.success = true;
    res.message = "No Points Selected, use Geometric Axis";
    return true;
  }


  // get all indices from provided mesh vertices along path
  // TODO replace getRegionOfInterest with a path equivalent
  PathSelector sel;
  res.mesh_indices = sel.findPointsAlongSegments(req.input_mesh, points);

  if (!res.mesh_indices.empty())
  {
    res.success = true;
    res.message = "Selection complete";
  }
  else
  {
    res.success = false;
    res.message = "Unable to identify points along selected path";
  }

  return true;
}

bool PathSelectionArtist::transformPoint(const geometry_msgs::PointStamped::ConstPtr pt_stamped,
                                         geometry_msgs::Point& transformed_pt)
{
  // Get the current transform from the world frame to the frame of the sensor data
  tf::StampedTransform frame;
  try
  {
    listener_->lookupTransform(sensor_frame_, world_frame_, ros::Time(0), frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Eigen::Affine3d transform;
  tf::transformTFToEigen(frame, transform);

  // Transform the current selection point
  Eigen::Vector3d pt_vec_sensor;
  Eigen::Vector3d pt_vec_ff(pt_stamped->point.x, pt_stamped->point.y, pt_stamped->point.z);
  pt_vec_sensor = transform * pt_vec_ff;

  transformed_pt.x = pt_vec_sensor(0);
  transformed_pt.y = pt_vec_sensor(1);
  transformed_pt.z = pt_vec_sensor(2);

  return true;
}

void PathSelectionArtist::addSelectionPoint(const geometry_msgs::PointStampedConstPtr pt_stamped)
{
  if (!enabled_)
    return;

  geometry_msgs::Point pt;
  if (!transformPoint(pt_stamped, pt))
  {
    ROS_ERROR("addSelectionPoint() couldn't transformPoint");
    return;
  }

  // Get the iterator to the points and lines markers in the interactive marker
  std::vector<visualization_msgs::Marker>::iterator points_it;
  points_it = std::find_if(marker_array_.markers.begin(),
                           marker_array_.markers.end(),
                           [](const visualization_msgs::Marker& marker) { return marker.id == 0; });

  std::vector<visualization_msgs::Marker>::iterator lines_it;
  lines_it = std::find_if(marker_array_.markers.begin(),
                          marker_array_.markers.end(),
                          [](const visualization_msgs::Marker& marker) { return marker.id == 1; });

  // Add new point to the points marker
  if (points_it == marker_array_.markers.end() || lines_it == marker_array_.markers.end())
  {
    ROS_ERROR("addSelectionPoint() Unable to find line or point marker");
    return;
  }
  else
  {
    points_it->points.push_back(pt);
    lines_it->points.push_back(pt);
  }

  marker_pub_.publish(marker_array_);
}

pcl::PolygonMesh PathSelectionArtist::filterMesh(const pcl::PolygonMesh& input_mesh,
						 const std::vector<int>& inlying_indices)
{
  // mark inlying points as true and outlying points as false
  std::vector<bool> whitelist(input_mesh.cloud.width * input_mesh.cloud.height, false);
  for (std::size_t i = 0; i < inlying_indices.size(); ++i)
  {
    whitelist[static_cast<std::size_t>(inlying_indices[i])] = true;
  }

  // All points marked 'false' in the whitelist will be removed. If
  // all of the polygon's points are marked 'true', that polygon
  // should be included.
  pcl::PolygonMesh intermediate_mesh;
  intermediate_mesh.cloud = input_mesh.cloud;
  for (std::size_t i = 0; i < input_mesh.polygons.size(); ++i)
  {
    if (whitelist[input_mesh.polygons[i].vertices[0]] && whitelist[input_mesh.polygons[i].vertices[1]] &&
        whitelist[input_mesh.polygons[i].vertices[2]])
    {
      intermediate_mesh.polygons.push_back(input_mesh.polygons[i]);
    }
  }

  // Remove unused points and save the result to the output mesh
  pcl::surface::SimplificationRemoveUnusedVertices simplifier;
  pcl::PolygonMesh output_mesh;
  simplifier.simplify(intermediate_mesh, output_mesh);

  return output_mesh;
}

void PathSelectionArtist::enable(bool value) { enabled_ = value; }

}  // namespace opp_path_selection
