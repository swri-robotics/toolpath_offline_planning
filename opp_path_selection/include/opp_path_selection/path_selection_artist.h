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

#ifndef OPP_PATH_SELECTION_SELECTION_PATH_ARTIST_H
#define OPP_PATH_SELECTION_SELECTION_PATH_ARTIST_H

#include <pcl/PolygonMesh.h>

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <opp_msgs/GetPathSelectionMesh.h>
#include <opp_msgs/GetPathSelectionCloud.h>

namespace opp_path_selection
{
/**
 * @brief The PathSelectionArtist class uses the Publish Points plugin in Rviz to allow the user to drop points on a
 * mesh or geometry primitive, creating a closed polyline path. The path is displayed using an Interactive Marker, which
 * enables the user to reset the path. This class also contains a ROS service server to output the last selection
 * points.
 */
class PathSelectionArtist
{
public:
  /**
   * @brief PathSelectionArtist is the class constructor which initializes ROS communication objects and private
   * variables.The 'world_frame' argument is the highest-level fixed frame (i.e. "map", "odom", or "world"). The
   * "sensor_frame" argument is the aggregated data frame (typically the base frame of the kinematic chain, i.e
   * rail_base_link or robot_base_link)
   * @param nh
   * @param world_frame
   * @param sensor_frame
   */
  PathSelectionArtist(const ros::NodeHandle& nh, const std::string& world_frame, const std::string& sensor_frame);

  bool collectPath(const shape_msgs::Mesh& mesh_msg, std::vector<int>& points_idx, std::string& message);

  bool collectPathMesh(const shape_msgs::Mesh& mesh_msg, std::vector<int>& points_idx, std::string& message);

  void enable(bool value);

  bool clearPathPointsCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res);

protected:

  void getSensorData(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void addSelectionPoint(const geometry_msgs::PointStamped::ConstPtr pt_stamped);

  bool transformPoint(const geometry_msgs::PointStamped::ConstPtr pt_stamped, geometry_msgs::Point& transformed_pt);

  bool collectPathPointsCloudCb(opp_msgs::GetPathSelectionCloud::Request& req,
                                opp_msgs::GetPathSelectionCloud::Response& res);

  bool collectPathPointsMeshCb(opp_msgs::GetPathSelectionMesh::Request& req,
                               opp_msgs::GetPathSelectionMesh::Response& res);

  pcl::PolygonMesh filterMesh(const pcl::PolygonMesh& input_mesh, const std::vector<int>& inlying_indices);

  ros::NodeHandle nh_;

  std::string world_frame_;

  std::string sensor_frame_;

  ros::Subscriber drawn_points_sub_;

  ros::Publisher marker_pub_;

  ros::ServiceServer clear_path_points_srv_;

  ros::ServiceServer collect_path_points_srv_;

  std::shared_ptr<tf::TransformListener> listener_;

  visualization_msgs::MarkerArray marker_array_;

  bool enabled_;
};

}  // namespace opp_path_selection

#endif  // OPP_PATH_SELECTION_SELECTION_PATH_ARTIST_H
