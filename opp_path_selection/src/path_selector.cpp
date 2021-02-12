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

#include "opp_path_selection/path_selector.h"
#include "opp_path_selection/filter_impl.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/optional.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/region_growing.h>

namespace opp_path_selection
{

std::vector<int> PathSelector::findPointsAlongSegments(const shape_msgs::Mesh& input_mesh,
                                                       const std::vector<Eigen::Vector3d>& points)

{
  // Check size of selection points vector
  if (points.size() < 2)
  {
    ROS_ERROR("Must have at least 2 points in Path");
    return {};
  }
  // find id of every point from marker array, needed for Dijkstras
  std::vector<int> point_seg_indices;
  for (int i = 0; i < points.size(); i++)
  {
    double mind = std::numeric_limits<double>::max();
    int minj = -1;
    // TODO figure out why dmin is not equal to zero
    for (int j = 0; j < input_mesh.vertices.size(); j++)
    {
      Eigen::Vector3d pt2(input_mesh.vertices[j].x, input_mesh.vertices[j].y, input_mesh.vertices[j].z);
      Eigen::Vector3d D = pt2 - points[i];
      if (D.norm() < mind)
      {
        mind = D.norm();
        minj = j;
      }
    }
    point_seg_indices.push_back(minj);
  }

  // create a graph with the mesh (triangles and vertices)
  MeshGraph G;
  for (int i = 0; i < input_mesh.triangles.size(); i++)  // for each triangle in mesh, add each edge to graph
  {
    int v1 = input_mesh.triangles[i].vertex_indices[0];
    int v2 = input_mesh.triangles[i].vertex_indices[1];
    int v3 = input_mesh.triangles[i].vertex_indices[2];
    Eigen::Vector3d p1(input_mesh.vertices[v1].x, input_mesh.vertices[v1].y, input_mesh.vertices[v1].z);
    Eigen::Vector3d p2(input_mesh.vertices[v2].x, input_mesh.vertices[v2].y, input_mesh.vertices[v2].z);
    Eigen::Vector3d p3(input_mesh.vertices[v3].x, input_mesh.vertices[v3].y, input_mesh.vertices[v3].z);
    double d1 = (p1 - p2).norm();
    double d2 = (p2 - p3).norm();
    double d3 = (p3 - p1).norm();
    bool rtn;
    MeshGraph::edge_descriptor e1;
    boost::tie(e1, rtn) = boost::add_edge(v1, v2, d1, G);
    boost::tie(e1, rtn) = boost::add_edge(v2, v3, d2, G);
    boost::tie(e1, rtn) = boost::add_edge(v3, v1, d3, G);
  }  // end of for each triangle in mesh
  std::vector<int> path_indices;
  for (int i = 1; i < point_seg_indices.size(); i++)  // for each source path segment
  {
    // use dijkstras to find shortest connected path between every line segement defined by points
    std::vector<double> d;
    std::vector<vertex_des> p;
    size_t num_verts = num_vertices(G);
    size_t num_edges = boost::num_edges(G);
    p.resize(num_verts);
    d.resize(num_verts);
    boost::property_map<MeshGraph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, G);
    boost::property_map<MeshGraph, boost::vertex_index_t>::type index_map = get(boost::vertex_index, G);
    vertex_des start_vertex = vertex(point_seg_indices[i - 1], G);
    boost::dijkstra_shortest_paths(
        G,             // const Graph& g
        start_vertex,  // vertex_descriptor s, source, all shortest path distances computed from here
        &p[0],         // predecessorMap each vertex p[i] points the next closer one p[p[i]] the next after
        &d[0],         // distanceMap result providing the distance to each vertex from s
        weightmap,     // weightMap accesses the weight of each edge
        index_map,     // takes a vertex descriptor and finds the index of the vertex
        std::less<double>(),
        boost::closed_plus<double>(),
	(std::numeric_limits<double>::max)(),
        0.0,
        boost::default_dijkstra_visitor());

    int V = point_seg_indices[i];
    int start_index = point_seg_indices[i - 1];
    int q = 0;
    // follow predicesor chain from end endex to start index, pushing each one onto the
    std::vector<int> seg_indices;
    while (V != start_index && q < p.size())
    {
      seg_indices.push_back(V);
      V = p[V];
      q++;
    }
    // TODO check that q != p.size(), in this case, there is no path between the two points in the graph
    seg_indices.push_back(V);

    // reverse order of each segment
    for (int j = seg_indices.size() - 1; j >= 0; j--)
    {
      path_indices.push_back(seg_indices[j]);
    }
  }  // end of for each source path segment
  return path_indices;
}

std::vector<int> PathSelector::findPointsAlongSegments(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                       const std::vector<Eigen::Vector3d>& points)
{
  // Check size of selection points vector
  if (points.size() < 2)
  {
    ROS_ERROR("Must have at least 2 points in Path");
    return {};
  }

  // Check the size of the input point cloud
  if (input_cloud->points.size() == 0)
  {
    ROS_ERROR("No points to search for path selection");
    return {};
  }

  // create a kdtree to find nearest neighbors in cloud
  // use 8 nearest neighbors of each point to create a connected graph
  // use dijkstras on that graph to find shortest path on the surface between each segment end-poing

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(input_cloud->makeShared());
  int K = 8;
  MeshGraph G;

  // for each point in the point cloud, find its neighbors and create a graph with edges to every neighbor
  for (size_t p_index = 0; p_index < input_cloud->points.size(); p_index++)
  {
    std::vector<int> n_indices;      // neighbor indices
    std::vector<float> n_distances;  // distance from vertex to neighbor
    pcl::PointXYZ P;
    P.x = input_cloud->points[p_index].x;
    P.y = input_cloud->points[p_index].y;
    P.z = input_cloud->points[p_index].z;
    kdtree.nearestKSearch(P, K, n_indices, n_distances);
    for (size_t i = 0; i < n_indices.size(); i++)  // for each triangle in mesh, add each edge to graph
    {
      if (n_indices[i] > p_index)  // only add edge if index of neighbor is greater than current point's index
      {
        pcl::PointXYZ P2;
        P2.x = input_cloud->points[n_indices[i]].x;
        P2.x = input_cloud->points[n_indices[i]].y;
        P2.x = input_cloud->points[n_indices[i]].z;
        Eigen::Vector3d p1(P.x, P.y, P.z);
        Eigen::Vector3d p2(input_cloud->points[n_indices[i]].x,
                           input_cloud->points[n_indices[i]].y,
                           input_cloud->points[n_indices[i]].z);
        double distance = (p1 - p2).norm();

        bool rtn;
        MeshGraph::edge_descriptor e1;
        boost::tie(e1, rtn) = boost::add_edge(p_index, n_indices[i], distance, G);
      }  // end only add edge if index of neighbor is greater than current point's index
    }    // end of for each neighbor
  }      // end for each point in cloud
  // graph G has now been created.

  std::vector<int> point_seg_indices;
  for (int i = 0; i < points.size(); i++)
  {
    double mind = std::numeric_limits<double>::max();
    int minj = -1;
    // TODO figure out why dmin is not equal to zero
    for (int j = 0; j < input_cloud->points.size(); j++)
    {
      Eigen::Vector3d pt2(input_cloud->points[j].x, input_cloud->points[j].y, input_cloud->points[j].z);
      Eigen::Vector3d D = pt2 - points[i];
      if (D.norm() < mind)
      {
        mind = D.norm();
        minj = j;
      }
    }
    point_seg_indices.push_back(minj);
  }

  // find id of every point from marker array, needed for Dijkstras
  // TODO this is the exact same code as above in other implementation, it should be a subroutine
  std::vector<int> path_indices;
  for (int i = 1; i < point_seg_indices.size(); i++)  // for each source path segment
  {
    // use dijkstras to find shortest connected path between every line segement defined by points
    std::vector<double> d;
    std::vector<vertex_des> p;
    size_t num_verts = num_vertices(G);
    size_t num_edges = boost::num_edges(G);
    p.resize(num_verts);
    d.resize(num_verts);
    boost::property_map<MeshGraph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, G);
    boost::property_map<MeshGraph, boost::vertex_index_t>::type index_map = get(boost::vertex_index, G);
    vertex_des start_vertex = vertex(point_seg_indices[i - 1], G);
    boost::dijkstra_shortest_paths(
        G,             // const Graph& g
        start_vertex,  // vertex_descriptor s, source, all shortest path distances computed from here
        &p[0],         // predecessorMap each vertex p[i] points the next closer one p[p[i]] the next after
        &d[0],         // distanceMap result providing the distance to each vertex from s
        weightmap,     // weightMap accesses the weight of each edge
        index_map,     // takes a vertex descriptor and finds the index of the vertex
        std::less<double>(),
        boost::closed_plus<double>(),
	(std::numeric_limits<double>::max)(),
        0.0,
        boost::default_dijkstra_visitor());

    int V = point_seg_indices[i];
    int start_index = point_seg_indices[i - 1];
    int q = 0;
    // follow predicesor chain from end endex to start index, pushing each one onto the
    std::vector<int> seg_indices;
    while (V != start_index && q < p.size())
    {
      seg_indices.push_back(V);
      V = p[V];
      q++;
    }
    // TODO check that q != p.size(), in this case, there is no path between the two points in the graph
    seg_indices.push_back(V);

    // reverse order of each segment
    for (int j = seg_indices.size() - 1; j >= 0; j--)
    {
      path_indices.push_back(seg_indices[j]);
    }
  }  // end of for each source path segment
  return path_indices;
}

}  // namespace opp_path_selection
