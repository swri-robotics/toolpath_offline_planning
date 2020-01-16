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

#ifndef OPP_GUI_WIDGETS_TOOL_PATH_PLANNER_WIDGET_H
#define OPP_GUI_WIDGETS_TOOL_PATH_PLANNER_WIDGET_H

#include <QSqlTableModel>
#include <QWidget>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service.h>

#include <opp_database/opp_database_interface_cpp.h>
#include <opp_msgs/Part.h>

class QListWidgetItem;

namespace Ui
{
class ToolPathPlanner;
}

namespace opp_gui
{
class TouchPointEditorWidget;
class ToolPathEditorWidget;

/**
 * @brief A widget for facilitating the definition of a new part (including specification of
 * geometry, touch points, and verification points) and the generation of tool paths
 * associated with that part. This widget interfaces with the application database to save
 * the part information for future use.
 */
class ToolPathPlannerWidget : public QWidget
{
  Q_OBJECT
public:
  ToolPathPlannerWidget(QWidget* parent = nullptr,
                        const ros::NodeHandle& nh = ros::NodeHandle("~"),
                        const std::vector<std::string>& frames = { "map" });

protected Q_SLOTS:

  void setVisualizationFrame(const QString& text);

  // Parts Page
  void browseForMeshResource();
  void loadMeshFromResource();

  void loadModelsFromDatabase();
  void onModelSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous);
  void loadSelectedModel();
  void saveModel();

  // Jobs Page
  void newJob();
  void loadJobsFromDatabase();
  void onJobSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous);
  void loadSelectedJob();
  void saveJob();

  // Database Management Page
  void showPartFromDatabase();
  void deletePart();
  void deleteJob();
  void refresh();

private:
  void clear();

  bool loadMesh();

  void setModelTabsEnabled(bool enabled);

  void setJobTabsEnabled(bool enabled, bool first_enabled = true);

  Ui::ToolPathPlanner* ui_;

  TouchPointEditorWidget* touch_point_editor_;
  TouchPointEditorWidget* verification_point_editor_;
  ToolPathEditorWidget* tool_path_editor_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::ServiceClient save_model_client_;
  ros::ServiceClient save_job_client_;
  ros::ServiceClient get_all_models_client_;

  std::string marker_frame_;

  std::vector<opp_msgs::Part> existing_parts_;
  std::vector<opp_msgs::Job> existing_jobs_;

  std::string mesh_resource_;
  uint32_t generated_model_id_;

  opp_db::ROSDatabaseInterface database_;
  QSqlTableModel* model_parts_;
  QSqlTableModel* model_jobs_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOOL_PATH_PLANNER_WIDGET_H
