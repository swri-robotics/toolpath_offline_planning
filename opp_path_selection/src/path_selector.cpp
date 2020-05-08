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

#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/region_growing.h>

#include "opp_path_selection/filter.h"

namespace opp_path_selection
{
int PathSelector::pointToLineSegmentDistance(const Eigen::Vector3d& segStart,
				 const Eigen::Vector3d& segEnd,
				 const Eigen::Vector3d& pt,
				 double& distance_to_seg,
				 double& distance_along_seg)
{
    int rtn = 1;
    Eigen::Vector3d line = segEnd-segStart;
    Eigen::Vector3d ptln = pt-segStart;
    double seg_len = line.norm();
    line.normalize();
    
    distance_to_seg =  line.cross(ptln).norm();
    distance_along_seg = line.dot(ptln);
    if(distance_along_seg<0)
      {
	Eigen::Vector3d dv = segStart - pt;
	distance_to_seg =  dv.norm();
	rtn = 0;
      }
    else if(distance_along_seg>seg_len)
      {
	Eigen::Vector3d dv = segEnd - pt;
	distance_to_seg =  dv.norm();
	rtn= -1;
      }
    return rtn;
  }


std::vector<int>  PathSelector::findPointsAlongSegments(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
							  const std::vector<Eigen::Vector3d>& points,
							  const PathSelectorParameters& params)
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

  // Create a list of vertices and their distances
  std::list<std::pair<double, int> > id_list;
  for (auto it = points.begin() + 1; it != points.end(); ++it)
  {
    Eigen::Vector3d seg_start = *(it-1);
    Eigen::Vector3d seg_end = *it;

    for (int i = 0; i < (int) input_cloud->points.size(); i++)
      {
	double distance_to_seg, distance_along_seg;
	Eigen::Vector3d pt(input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z);
	int rtn = pointToLineSegmentDistance(seg_start, seg_end, pt, distance_to_seg, distance_along_seg);
	if(rtn == 1 && distance_to_seg < params.line_threshold)
	  {
	    id_list.push_back(std::pair<double,int>(distance_along_seg,i));
	  }	
      }// end for each point in cloud
  }// for each path segment
  
  id_list.sort();

  std::vector<int> path_indices;
  for(auto a = id_list.begin(); a!=id_list.end(); a++)
    {
      path_indices.push_back(a->second);
    }
  
  return path_indices;
}


}  // namespace opp_path_selection
