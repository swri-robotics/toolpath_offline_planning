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

#ifndef OPP_MSGS_SERIALIZATION_OPP_MSGS_YAML_H
#define OPP_MSGS_SERIALIZATION_OPP_MSGS_YAML_H

#include <noether_msgs/ToolPathConfig.h>

#include <opp_msgs/Job.h>
#include <opp_msgs/ToolPath.h>
#include <opp_msgs/TouchPoint.h>
#include <message_serialization/geometry_msgs_yaml.h>
#include <message_serialization/sensor_msgs_yaml.h>

namespace YAML
{
template <>
struct convert<noether_msgs::ToolPathConfig>
{
  static Node encode(const noether_msgs::ToolPathConfig& rhs)
  {
    Node node;

    node["surface_walk_generator"] = rhs.surface_walk_generator;
    node["plane_slicer_generator"] = rhs.plane_slicer_generator;
    node["heat_generator"] = rhs.heat_generator;
    node["eigen_value_generator"] = rhs.eigen_value_generator;
    node["half_edge_generator"] = rhs.halfedge_generator;
    return node;
  }

  static bool decode(const Node& node, noether_msgs::ToolPathConfig& rhs)
  {
    if (node.size() != 5)
      return false;
    rhs.plane_slicer_generator = node["plane_slicer_generator"].as<decltype(rhs.plane_slicer_generator)>();
    rhs.halfedge_generator = node["half_edge_generator"].as<decltype(rhs.halfedge_generator)>();
    rhs.eigen_value_generator = node["eigen_value_generator"].as<decltype(rhs.eigen_value_generator)>();
    rhs.heat_generator = node["heat_generator"].as<decltype(rhs.heat_generator)>();
    rhs.surface_walk_generator = node["surface_walk_generator"].as<decltype(rhs.surface_walk_generator)>();
    return true;
  }
};

template <>
struct convert<noether_msgs::HalfedgeEdgeGeneratorConfig>
{
  static Node encode(const noether_msgs::HalfedgeEdgeGeneratorConfig& rhs)
  {
    Node node;

    node["min_num_points"] = rhs.min_num_points;
    node["normal_averaging"] = rhs.normal_averaging;
    node["normal_search_radius"] = rhs.normal_search_radius;
    node["normal_influence_weight"] = rhs.normal_influence_weight;
    node["point_spacing_method"] = rhs.point_spacing_method;
    node["point_dist"] = rhs.point_dist;
    node["max_segment_length"] = rhs.max_segment_length;
    return node;
  }

  static bool decode(const Node& node, noether_msgs::HalfedgeEdgeGeneratorConfig& rhs)
  {
    if (node.size() != 7)
      {
	return false;
      }
    int q=0;

    rhs.min_num_points = node["min_num_points"].as<decltype(rhs.min_num_points)>();
    rhs.normal_averaging = node["normal_averaging"].as<decltype(rhs.normal_averaging)>();
    rhs.normal_search_radius = node["normal_search_radius"].as<decltype(rhs.normal_search_radius)>();
    rhs.normal_influence_weight = node["normal_influence_weight"].as<decltype(rhs.normal_influence_weight)>();
    rhs.point_spacing_method = node["point_spacing_method"].as<decltype(rhs.point_spacing_method)>();
    rhs.point_dist = node["point_dist"].as<decltype(rhs.point_dist)>();
    rhs.max_segment_length = node["max_segment_length"].as<decltype(rhs.max_segment_length)>();
    return true;
  }
};

template <>
struct convert<noether_msgs::SurfaceWalkRasterGeneratorConfig>
{
  static Node encode(const noether_msgs::SurfaceWalkRasterGeneratorConfig& rhs)
  {
    Node node;

    node["point_spacing"] = rhs.point_spacing;
    node["raster_spacing"] = rhs.raster_spacing;
    node["tool_offset"] = rhs.tool_offset;
    node["intersection_plane_height"] = rhs.intersection_plane_height;
    node["min_hole_size"] = rhs.min_hole_size;
    node["min_segment_size"] = rhs.min_segment_size;
    node["raster_rot_offset"] = rhs.raster_rot_offset;
    node["generate_extra_rasters"] = rhs.generate_extra_rasters;
    node["cut_direction"] = rhs.cut_direction;
    return node;
  }

  static bool decode(const Node& node, noether_msgs::SurfaceWalkRasterGeneratorConfig& rhs)
  {
    if (node.size() != 9)
      return false;
    rhs.raster_spacing = node["raster_spacing"].as<decltype(rhs.raster_spacing)>();
    rhs.point_spacing = node["point_spacing"].as<decltype(rhs.point_spacing)>();
    rhs.tool_offset = node["tool_offset"].as<decltype(rhs.tool_offset)>();
    rhs.intersection_plane_height = node["intersection_plane_height"].as<decltype(rhs.intersection_plane_height)>();
    rhs.min_hole_size = node["min_hole_size"].as<decltype(rhs.min_hole_size)>();
    rhs.min_segment_size = node["min_segment_size"].as<decltype(rhs.min_segment_size)>();
    rhs.raster_rot_offset = node["raster_rot_offset"].as<decltype(rhs.raster_rot_offset)>();
    rhs.generate_extra_rasters = node["generate_extra_rasters"].as<decltype(rhs.generate_extra_rasters)>();
    rhs.cut_direction = node["cut_direction"].as<decltype(rhs.cut_direction)>();
    return true;
  }
};

template <>
struct convert<noether_msgs::PlaneSlicerRasterGeneratorConfig>
{
  static Node encode(const noether_msgs::PlaneSlicerRasterGeneratorConfig& rhs)
  {
    Node node;
    node["raster_spacing"] = rhs.raster_spacing;
    node["point_spacing"] = rhs.point_spacing;
    node["raster_rot_offset"] = rhs.raster_rot_offset;
    node["min_segment_size"] = rhs.min_segment_size;
    node["min_hole_size"] = rhs.min_hole_size;
    node["tool_offset"] = rhs.tool_offset;
    node["raster_wrt_global_axes"] = rhs.raster_wrt_global_axes;
    node["raster_direction"] = rhs.raster_direction;
    return node;
  }

  static bool decode(const Node& node, noether_msgs::PlaneSlicerRasterGeneratorConfig& rhs)
  {
    if (node.size() != 8)
      return false;

    rhs.raster_spacing = node["raster_spacing"].as<decltype(rhs.raster_spacing)>();
    rhs.point_spacing = node["point_spacing"].as<decltype(rhs.point_spacing)>();
    rhs.raster_rot_offset = node["raster_rot_offset"].as<decltype(rhs.raster_rot_offset)>();
    rhs.min_segment_size = node["min_segment_size"].as<decltype(rhs.min_segment_size)>();
    rhs.min_hole_size = node["min_hole_size"].as<decltype(rhs.min_hole_size)>();
    rhs.tool_offset = node["tool_offset"].as<decltype(rhs.tool_offset)>();
    rhs.raster_wrt_global_axes = node["raster_wrt_global_axes"].as<decltype(rhs.raster_wrt_global_axes)>();
    rhs.raster_direction = node["raster_direction"].as<decltype(rhs.raster_direction)>();
    return true;
  }
};

template <>
struct convert<heat_msgs::HeatRasterGeneratorConfig>
{
  static Node encode(const heat_msgs::HeatRasterGeneratorConfig& rhs)
  {
    Node node;

    node["point_spacing"] = rhs.point_spacing;
    node["raster_spacing"] = rhs.raster_spacing;
    node["tool_offset"] = rhs.tool_offset;
    node["min_hole_size"] = rhs.min_hole_size;
    node["min_segment_size"] = rhs.min_segment_size;
    node["raster_rot_offset"] = rhs.raster_rot_offset;
    node["generate_extra_rasters"] = rhs.generate_extra_rasters;
    return node;
  }

  static bool decode(const Node& node, heat_msgs::HeatRasterGeneratorConfig& rhs)
  {
    if (node.size() != 7)
      return false;
    rhs.point_spacing = node["point_spacing"].as<decltype(rhs.point_spacing)>();
    rhs.raster_spacing = node["raster_spacing"].as<decltype(rhs.raster_spacing)>();
    rhs.tool_offset = node["tool_offset"].as<decltype(rhs.tool_offset)>();
    rhs.min_hole_size = node["min_hole_size"].as<decltype(rhs.min_hole_size)>();
    rhs.min_segment_size = node["min_segment_size"].as<decltype(rhs.min_segment_size)>();
    rhs.raster_rot_offset = node["raster_rot_offset"].as<decltype(rhs.raster_rot_offset)>();
    rhs.generate_extra_rasters = node["generate_extra_rasters"].as<decltype(rhs.generate_extra_rasters)>();
    return true;
  }
};

template <>
struct convert<noether_msgs::EigenValueEdgeGeneratorConfig>
{
  static Node encode(const noether_msgs::EigenValueEdgeGeneratorConfig& rhs)
  {
    Node node;

    node["octree_res"] = rhs.octree_res;
    node["search_radius"] = rhs.search_radius;
    node["num_threads"] = rhs.num_threads;
    node["neighbor_tol"] = rhs.neighbor_tol;
    node["voxel_size"] = rhs.voxel_size;
    node["edge_cluster_min"] = rhs.edge_cluster_min;
    node["kdtree_epsilon"] = rhs.kdtree_epsilon;
    node["min_projection_dist"] = rhs.min_projection_dist;
    node["max_intersecting_voxels"] = rhs.max_intersecting_voxels;
    node["merge_dist"] = rhs.merge_dist;
    node["max_segment_length"] = rhs.max_segment_length;
    return node;
  }

  static bool decode(const Node& node, noether_msgs::EigenValueEdgeGeneratorConfig& rhs)
  {
    if (node.size() != 11)
      return false;
    rhs.octree_res = node["octree_res"].as<decltype(rhs.octree_res)>();
    rhs.search_radius = node["search_radius"].as<decltype(rhs.search_radius)>();
    rhs.num_threads = node["num_threads"].as<decltype(rhs.num_threads)>();
    rhs.neighbor_tol = node["neighbor_tol"].as<decltype(rhs.neighbor_tol)>();
    rhs.voxel_size = node["voxel_size"].as<decltype(rhs.voxel_size)>();
    rhs.edge_cluster_min = node["edge_cluster_min"].as<decltype(rhs.edge_cluster_min)>();
    rhs.kdtree_epsilon = node["kdtree_epsilon"].as<decltype(rhs.kdtree_epsilon)>();
    rhs.min_projection_dist = node["min_projection_dist"].as<decltype(rhs.min_projection_dist)>();
    rhs.max_intersecting_voxels = node["max_intersecting_voxels"].as<decltype(rhs.max_intersecting_voxels)>();
    rhs.merge_dist = node["merge_dist"].as<decltype(rhs.merge_dist)>();
    rhs.max_segment_length = node["max_segment_length"].as<decltype(rhs.max_segment_length)>();
    return true;
  }
};

template <>
struct convert<opp_msgs::Job>
{
  static Node encode(const opp_msgs::Job& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["id"] = rhs.id;
    node["name"] = rhs.name;
    node["description"] = rhs.description;
    node["part_id"] = rhs.part_id;
    node["paths"] = rhs.paths;
    return node;
  }

  static bool decode(const Node& node, opp_msgs::Job& rhs)
  {
    if (node.size() != 6)
      return false;
    rhs.header = node["header"].as<decltype(rhs.header)>();
    rhs.id = node["id"].as<decltype(rhs.id)>();
    rhs.name = node["name"].as<decltype(rhs.name)>();
    rhs.description = node["description"].as<decltype(rhs.description)>();
    rhs.part_id = node["part_id"].as<decltype(rhs.part_id)>();
    rhs.paths = node["paths"].as<decltype(rhs.paths)>();
    return true;
  }
};

template <>
struct convert<opp_msgs::ProcessType>
{
  static Node encode(const opp_msgs::ProcessType& rhs)
  {
    Node node;
    node["val"] = rhs.val;
    return node;
  }

  static bool decode(const Node& node, opp_msgs::ProcessType& rhs)
  {
    if (node.size() != 1)
      return false;
    rhs.val = node["val"].as<decltype(rhs.val)>();
    return true;
  }
};

template <>
struct convert<opp_msgs::ToolPath>
{
  static Node encode(const opp_msgs::ToolPath& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["process_type"] = rhs.process_type;
    node["paths"] = rhs.paths;
    node["dwell_time"] = rhs.dwell_time;
    node["tool_offset"] = rhs.tool_offset;
    node["params"] = rhs.params;
    return node;
  }

  static bool decode(const Node& node, opp_msgs::ToolPath& rhs)
  {
    // Allow both old entries (without Noether params) and new entries (with Noether params)
    //    if (node.size() != 6)
    //    {
    //      return false;
    //    }
    // Get the opp_msgs::ToolPath fields
    rhs.header = node["header"].as<decltype(rhs.header)>();
    rhs.process_type = node["process_type"].as<decltype(rhs.process_type)>();
    rhs.paths = node["paths"].as< std::vector<geometry_msgs::PoseArray> >();
    rhs.dwell_time = node["dwell_time"].as<decltype(rhs.dwell_time)>();
    rhs.tool_offset = node["tool_offset"].as<decltype(rhs.tool_offset)>();
    if (node["params"])
    {
      rhs.params = node["params"].as<decltype(rhs.params)>();
    }
    return true;
  }
};

template <>
struct convert<opp_msgs::ToolPathParams>
{
  static Node encode(const opp_msgs::ToolPathParams& rhs)
  {
    Node node;
    // Do the things inside the noether_msgs::ToolPathConfig
    node["config"] = rhs.config;
    node["curvature_threshold"] = rhs.curvature_threshold;
    node["min_polygons_per_cluster"] = rhs.min_polygons_per_cluster;
    return node;
  }

  static bool decode(const Node& node, opp_msgs::ToolPathParams& rhs)
  {
    if (node.size() != 3)
    {
      return false;
    }
    rhs.config = node["config"].as<decltype(rhs.config)>();
    rhs.curvature_threshold = node["curvature_threshold"].as<decltype(rhs.curvature_threshold)>();
    rhs.min_polygons_per_cluster = node["min_polygons_per_cluster"].as<decltype(rhs.min_polygons_per_cluster)>();
    return true;
  }
};

template <>
struct convert<opp_msgs::TouchPoint>
{
  static Node encode(const opp_msgs::TouchPoint& rhs)
  {
    Node node;
    node["name"] = rhs.name;
    node["description"] = rhs.description;
    node["threshold"] = rhs.threshold;
    node["transform"] = rhs.transform;
    return node;
  }

  static bool decode(const Node& node, opp_msgs::TouchPoint& rhs)
  {
    if (node.size() != 4)
      return false;
    rhs.name = node["name"].as<decltype(rhs.name)>();
    rhs.description = node["description"].as<decltype(rhs.description)>();
    rhs.threshold = node["threshold"].as<decltype(rhs.threshold)>();
    rhs.transform = node["transform"].as<decltype(rhs.transform)>();
    return true;
  }
};

}  // namespace YAML

#endif  // OPP_MSGS_SERIALIZATION_OPP_MSGS_YAML_H
