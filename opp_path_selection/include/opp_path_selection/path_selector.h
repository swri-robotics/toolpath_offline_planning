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

#ifndef OPP_PATH_SELECTION_PATH_SELECTOR_H
#define OPP_PATH_SELECTION_PATH_SELECTOR_H

#include <eigen3/Eigen/Core>

#include <pcl_ros/point_cloud.h>
#include <shape_msgs/Mesh.h>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace opp_path_selection
{
/**
 * @brief The PathSelector class takes a set of 3D points defining path segments and attempts to identify which
 * points in a published point cloud lie along those segements.
 */
class PathSelector
{
  
public:
  typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
  typedef boost::property<boost::vertex_color_t, boost::default_color_type> VertexColorProperty;
  typedef boost::adjacency_list<boost::listS,
                                boost::vecS,
                                boost::undirectedS,
                                boost::disallow_parallel_edge_tag,
                                EdgeWeightProperty,
                                VertexColorProperty>
      MeshGraph;
  typedef boost::property_map<MeshGraph, boost::vertex_index_t>::type IndexMap;
  typedef boost::graph_traits<MeshGraph>::edge_iterator edge_itr;
  typedef boost::graph_traits<MeshGraph>::edge_descriptor edge_desc;
  typedef boost::graph_traits<MeshGraph>::vertex_descriptor vertex_des;
  typedef boost::property_map<MeshGraph, boost::vertex_index_t> Vertex_id;

  /**
   * @brief PathSelector class constructor
   */
  PathSelector() = default;

  /**
   * @brief Finds the points that lie along the segments defined by the points
   * @param input_mesh
   * @return Returns false if there are less than 2 points in segment, or if less than 2 vertices are found
   * otherwise returns true.
   */
  std::vector<int> findPointsAlongSegments(const shape_msgs::Mesh& input_mesh,
                                           const std::vector<Eigen::Vector3d>& points);

  /**
   * @brief Finds the points that lie along the segments defined by the points
   * @param input_cloud point cloud of mesh on which the path is created
   * @return Returns false if there are less than 2 points in segment, or if less than 2 vertices are found
   * otherwise returns true.
   */
  std::vector<int> findPointsAlongSegments(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                           const std::vector<Eigen::Vector3d>& points);

protected:
};

}  // namespace opp_path_selection

#endif  // OPP_PATH_SELECTION_PATH_SELECTOR_H
