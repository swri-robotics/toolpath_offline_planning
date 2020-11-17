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

#include "opp_area_selection/selection_artist.h"

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

#include "opp_area_selection/area_selector.h"
#include "opp_area_selection/area_selector_parameters.h"
#include <opp_msgs/GetROISelection.h>

namespace opp_area_selection
{
const static double TF_LOOKUP_TIMEOUT = 5.0;
const static std::string MARKER_ARRAY_TOPIC = "roi_markers";
const static std::string CLICKED_POINT_TOPIC = "clicked_point";
const static std::string CLEAR_ROI_POINTS_SERVICE = "clear_selection_points";
const static std::string COLLECT_ROI_POINTS_SERVICE = "collect_selection_points";
const static std::string area_selection_config_file = ros::package::getPath("opp_area_selection") + "/config/"
                                                                                                    "area_selection_"
                                                                                                    "parameters.yaml";

}  // namespace opp_area_selection

namespace
{
std::vector<visualization_msgs::Marker> makeVisual(const std::string& frame_id)
{
  visualization_msgs::Marker points, lines;
  points.header.frame_id = lines.header.frame_id = frame_id;
  points.ns = lines.ns = "roi_selection";
  points.action = lines.action = visualization_msgs::Marker::ADD;

  // Point specific properties
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.scale.x = points.scale.y = points.scale.z = 0.025;
  points.color.r = points.color.a = 1.0;
  points.pose.orientation.w = 1.0;

  // Line specific properties
  lines.id = 1;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.scale.x = 0.01;
  lines.color.r = lines.color.a = 1.0;
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

namespace YAML
{
template <>
struct convert<opp_area_selection::AreaSelectorParameters>
{
  static Node encode(const opp_area_selection::AreaSelectorParameters& rhs)
  {
    Node node;
    node["cluster_tolerance"] = rhs.cluster_tolerance;
    node["min_cluster_size"] = rhs.min_cluster_size;
    node["max_cluster_size"] = rhs.max_cluster_size;
    node["plane_distance_threshold"] = rhs.plane_distance_threshold;
    node["normal_est_radius"] = rhs.normal_est_radius;
    node["region_growing_nneighbors"] = rhs.region_growing_nneighbors;
    node["region_growing_smoothness"] = rhs.region_growing_smoothness;
    node["region_growing_curvature"] = rhs.region_growing_curvature;
    return node;
  }

  static bool decode(const Node& node, opp_area_selection::AreaSelectorParameters& rhs)
  {
    if (node.size() != 8)
    {
      return false;
    }
    rhs.cluster_tolerance = node["cluster_tolerance"].as<decltype(rhs.cluster_tolerance)>();
    rhs.min_cluster_size = node["min_cluster_size"].as<decltype(rhs.min_cluster_size)>();
    rhs.max_cluster_size = node["max_cluster_size"].as<decltype(rhs.max_cluster_size)>();
    rhs.plane_distance_threshold = node["plane_distance_threshold"].as<decltype(rhs.plane_distance_threshold)>();
    rhs.normal_est_radius = node["normal_est_radius"].as<decltype(rhs.normal_est_radius)>();
    rhs.region_growing_nneighbors = node["region_growing_nneighbors"].as<decltype(rhs.region_growing_nneighbors)>();
    rhs.region_growing_smoothness = node["region_growing_smoothness"].as<decltype(rhs.region_growing_smoothness)>();
    rhs.region_growing_curvature = node["region_growing_curvature"].as<decltype(rhs.region_growing_curvature)>();
    return true;
  }
};

}  // namespace YAML

namespace opp_area_selection
{
SelectionArtist::SelectionArtist(const ros::NodeHandle& nh,
                                 const std::string& world_frame,
                                 const std::string& sensor_frame)
  : nh_(nh), world_frame_(world_frame), sensor_frame_(sensor_frame)
{
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(MARKER_ARRAY_TOPIC, 1, false);
  listener_.reset(new tf::TransformListener(nh_));

  if (!listener_->waitForTransform(sensor_frame_, world_frame_, ros::Time(0), ros::Duration(TF_LOOKUP_TIMEOUT)))
  {
    ROS_ERROR("Transform lookup from %s to %s timed out", sensor_frame_.c_str(), world_frame_.c_str());
    throw std::runtime_error("Transform lookup timed out");
  }

  clear_roi_points_srv_ = nh_.advertiseService(CLEAR_ROI_POINTS_SERVICE, &SelectionArtist::clearROIPointsCb, this);
  collect_roi_points_srv_ =
      nh_.advertiseService(COLLECT_ROI_POINTS_SERVICE, &SelectionArtist::collectROIPointsCb, this);

  // Initialize subscribers and callbacks
  boost::function<void(const geometry_msgs::PointStampedConstPtr&)> drawn_points_cb;
  drawn_points_cb = boost::bind(&SelectionArtist::addSelectionPoint, this, _1);
  drawn_points_sub_ = nh_.subscribe(CLICKED_POINT_TOPIC, 1, drawn_points_cb);

  marker_array_.markers = makeVisual(sensor_frame);

  enabled_ = true;
}

bool SelectionArtist::clearROIPointsCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  (void)req;  // To suppress warnings, tell the compiler we will not use this parameter

  for (auto it = marker_array_.markers.begin(); it != marker_array_.markers.end(); ++it)
  {
    it->points.clear();
  }

  marker_pub_.publish(marker_array_);
  res.success = true;
  res.message = "Selection cleared";

  return true;
}

bool SelectionArtist::collectROIMesh(const shape_msgs::Mesh& mesh_msg,
                                     shape_msgs::Mesh& submesh_msg,
                                     std::string& message)
{
  pcl::PolygonMesh mesh;
  pclFromShapeMsg(mesh_msg, mesh);
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
  opp_msgs::GetROISelection srv;
  pcl::toROSMsg(mesh_cloud, srv.request.input_cloud);

  bool success = collectROIPointsCb(srv.request, srv.response);
  if (!success || !srv.response.success)
  {
    submesh_msg = mesh_msg;
    message = srv.response.message;
    return false;
  }

  pcl::PolygonMesh submesh;
  filterMesh(mesh, srv.response.cloud_indices, submesh);
  pclToShapeMsg(submesh, submesh_msg);

  return true;
}

bool SelectionArtist::collectROIPointsCb(opp_msgs::GetROISelectionRequest& req, opp_msgs::GetROISelectionResponse& res)
{
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(req.input_cloud, *cloud);

  AreaSelectorParameters params;
  bool success = message_serialization::deserialize(area_selection_config_file, params);
  if (!success)
  {
    ROS_ERROR_STREAM("Could not load area selection config from: " << area_selection_config_file);
    return false;
  }
  AreaSelector sel;
  res.cloud_indices = sel.getRegionOfInterest(cloud, points, params);

  if (!res.cloud_indices.empty())
  {
    res.success = true;
    res.message = "Selection complete";
  }
  else
  {
    res.success = false;
    res.message = "Unable to identify points within selection boundary";
  }

  return true;
}

bool SelectionArtist::transformPoint(const geometry_msgs::PointStamped::ConstPtr pt_stamped,
                                     geometry_msgs::Point& transformed_pt)
{
  ROS_INFO_STREAM(pt_stamped->header.frame_id);
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

void SelectionArtist::addSelectionPoint(const geometry_msgs::PointStampedConstPtr pt_stamped)
{
  if (!enabled_)
    return;

  geometry_msgs::Point pt;
  if (!transformPoint(pt_stamped, pt))
  {
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
    ROS_ERROR("Unable to find line or point marker");
    return;
  }
  else
  {
    points_it->points.push_back(pt);

    // Add the point to the front and back of the lines' points array if it is the first entry
    // Lines connect adjacent points, so first point must be entered twice to close the polygon
    if (lines_it->points.empty())
    {
      lines_it->points.push_back(pt);
      lines_it->points.push_back(pt);
    }
    // Insert the new point in the second to last position if points already exist in the array
    else
    {
      const auto it = lines_it->points.end() - 1;
      lines_it->points.insert(it, pt);
    }
  }

  marker_pub_.publish(marker_array_);
}

void SelectionArtist::filterMesh(const pcl::PolygonMesh& input_mesh,
                                 const std::vector<int>& inlying_indices,
                                 pcl::PolygonMesh& output_mesh)
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
  simplifier.simplify(intermediate_mesh, output_mesh);

  return;
}

void SelectionArtist::enable(bool value) { enabled_ = value; }
}  // namespace opp_area_selection
