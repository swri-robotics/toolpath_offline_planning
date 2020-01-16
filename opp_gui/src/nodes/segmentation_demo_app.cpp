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

#include <pcl/io/vtk_lib_io.h>
#include <QApplication>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "opp_gui/utils.h"
#include "opp_gui/widgets/segmentation_parameters_editor_widget.h"

int main(int argc, char** argv)
{
  // Setup ROS Node
  ros::init(argc, argv, "segmentation_demo_app");
  ros::NodeHandle nh, pnh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Import the mesh
  std::string filename;
  bool save_outputs;
  pnh.param<std::string>("filename", filename, "");
  pnh.param<bool>("save_outputs", save_outputs, false);
  pcl::PolygonMesh pcl_mesh;
  pcl::io::loadPolygonFile(filename, pcl_mesh);
  ROS_INFO_STREAM("Imported as PCL mesh of size " << pcl_mesh.cloud.data.size() << '\n');

  // Convert to ROS message
  pcl_msgs::PolygonMesh pcl_mesh_msg;
  pcl_conversions::fromPCL(pcl_mesh, pcl_mesh_msg);

  // Create and start the Qt application
  QApplication app(argc, argv);

  // Start the widget
  opp_gui::SegmentationParametersEditorWidget* segmentation_widget =
      new opp_gui::SegmentationParametersEditorWidget(nullptr);
  segmentation_widget->show();
  segmentation_widget->init(pcl_mesh_msg);

  app.exec();

  // Save the meshes
  auto segmented_meshes = segmentation_widget->getSegments();
  if (save_outputs)
  {
    for (int ind = 0; ind < segmented_meshes.size(); ind++)
    {
      ROS_INFO_STREAM("Saving: " << ind << "\n");

      std::ostringstream ss;
      ss << ind;
      std::string filename =
          ros::package::getPath("opp_demos") + "/support/outputs/segmentation_output_" + ss.str() + ".stl";

      pcl_msgs::PolygonMesh pcl_msg_mesh;
      opp_gui::utils::pclMsgFromShapeMsg(*segmented_meshes[ind], pcl_mesh_msg);
      pcl::PolygonMesh pcl_mesh;
      pcl_conversions::toPCL(pcl_mesh_msg, pcl_mesh);
      pcl::io::savePolygonFile(filename, pcl_mesh);
    }
  }

  ros::waitForShutdown();

  delete segmentation_widget;

  return 0;
}
