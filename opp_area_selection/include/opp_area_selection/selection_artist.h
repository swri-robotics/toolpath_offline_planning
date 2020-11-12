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

#ifndef OPP_AREA_SELECTION_SELECTION_AREA_ARTIST_H
#define OPP_AREA_SELECTION_SELECTION_AREA_ARTIST_H

#include <pcl/PolygonMesh.h>

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <opp_msgs/GetROISelection.h>

namespace opp_area_selection
{
/**
 * @brief The SelectionArtist class uses the Publish Points plugin in Rviz to allow the user to drop points on a mesh or
 * geometry primitive, creating a closed polygon region-of-interest (ROI). The ROI is displayed using an Interactive
 * Marker, which enables the user to reset the ROI or display the points within the selection polygon by means of a
 * drop-down menu. This class also contains a ROS service server to output the last selection points.
 */
class SelectionArtist
{
public:
  /**
   * @brief SelectionArtist is the class constructor which initializes ROS communication objects and private
   * variables.The 'world_frame' argument is the highest-level fixed frame (i.e. "map", "odom", or "world"). The
   * "sensor_frame" argument is the aggregated data frame (typically the base frame of the kinematic chain, i.e
   * rail_base_link or robot_base_link)
   * @param nh
   * @param world_frame
   * @param sensor_frame
   */
  SelectionArtist(const ros::NodeHandle& nh, const std::string& world_frame, const std::string& sensor_frame);

  bool clearROIPointsCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  bool collectROIMesh(const shape_msgs::Mesh& mesh_msg, shape_msgs::Mesh& submesh_msg, std::string& message);

  void enable(bool value);

protected:
  void addSelectionPoint(const geometry_msgs::PointStamped::ConstPtr pt_stamped);

  bool transformPoint(const geometry_msgs::PointStamped::ConstPtr pt_stamped, geometry_msgs::Point& transformed_pt);

  bool collectROIPointsCb(opp_msgs::GetROISelection::Request& req, opp_msgs::GetROISelection::Response& res);

  void filterMesh(const pcl::PolygonMesh& input_mesh,
                  const std::vector<int>& inlying_indices,
                  pcl::PolygonMesh& output_mesh);

  ros::NodeHandle nh_;

  std::string world_frame_;

  std::string sensor_frame_;

  ros::Subscriber drawn_points_sub_;

  ros::Publisher marker_pub_;

  ros::ServiceServer clear_roi_points_srv_;

  ros::ServiceServer collect_roi_points_srv_;

  std::shared_ptr<tf::TransformListener> listener_;

  visualization_msgs::MarkerArray marker_array_;

  bool enabled_;
};

}  // namespace opp_area_selection

#endif  // OPP_AREA_SELECTION_SELECTION_AREA_ARTIST_H
