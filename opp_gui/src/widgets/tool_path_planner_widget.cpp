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

#include "opp_gui/widgets/tool_path_planner_widget.h"

#include <map>
#include <regex>
#include <string>

#include <QFileDialog>
#include <QMessageBox>
#include <QSqlField>
#include <QSqlRecord>
#include <QTableView>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>

#include "opp_gui/utils.h"
#include "opp_gui/widgets/tool_path_editor_widget.h"
#include "opp_gui/widgets/touch_point_editor_widget.h"
#include "ui_tool_path_planner.h"

const static std::string MESH_MARKER_TOPIC = "mesh_marker";
const static int MIN_TOUCH_POINTS = 0;
const static int MIN_VERIFICATION_POINTS = 0;

namespace opp_gui
{
ToolPathPlannerWidget::ToolPathPlannerWidget(QWidget* parent,
                                             const ros::NodeHandle& nh,
                                             const std::vector<std::string>& frames)
  : QWidget(parent), nh_(nh)
{
  ui_ = new Ui::ToolPathPlanner();
  ui_->setupUi(this);

  // Add the available tf frame
  if (!frames.empty())
  {
    marker_frame_ = frames.front();
  }
  else
  {
    ROS_ERROR_STREAM("No visualization frames available.  Using \"map\" instead.");
    marker_frame_ = "map";
  }

  // Create the other widgets
  touch_point_editor_ = new TouchPointEditorWidget(this, nh_, marker_frame_);
  verification_point_editor_ = new TouchPointEditorWidget(this, nh_, marker_frame_);
  tool_path_editor_ = new ToolPathEditorWidget(this, nh_, marker_frame_);

  // Set the color of the touch point markers
  touch_point_editor_->setMarkerColor(1.0, 0.0, 0.0);
  verification_point_editor_->setMarkerColor(0.0, 0.0, 1.0);

  // Add the widgets to the appropriate frames
  {
    // Touch point editor
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(touch_point_editor_);
    ui_->frame_define_touch_off_points->setLayout(layout);
  }
  {
    // Verification point editor
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(verification_point_editor_);
    ui_->frame_define_verification_points->setLayout(layout);
  }
  {
    // Tool path editor
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(tool_path_editor_);
    ui_->frame_define_toolpaths->setLayout(layout);
  }

  // Disable the "downstream" tabs until a model is loaded
  setModelTabsEnabled(false);
  setJobTabsEnabled(false, false);

  // Connect the signals and slots
  connect(ui_->push_button_find_model_file, &QPushButton::clicked, this, &ToolPathPlannerWidget::browseForMeshResource);
  connect(ui_->push_button_load_parts_from_database,
          &QPushButton::clicked,
          this,
          &ToolPathPlannerWidget::loadModelsFromDatabase);
  loadModelsFromDatabase();
  connect(
      ui_->list_widget_parts, &QListWidget::currentItemChanged, this, &ToolPathPlannerWidget::onModelSelectionChanged);
  connect(ui_->push_button_load_selected_part, &QPushButton::clicked, this, &ToolPathPlannerWidget::loadSelectedModel);
  connect(ui_->push_button_save_entry, &QPushButton::clicked, this, &ToolPathPlannerWidget::saveModel);

  // Signals & slots for the buttons on job definition page
  connect(ui_->push_button_new_job, &QPushButton::clicked, this, &ToolPathPlannerWidget::newJob);
  connect(ui_->push_button_load_jobs_from_database,
          &QPushButton::clicked,
          this,
          &ToolPathPlannerWidget::loadJobsFromDatabase);
  connect(ui_->list_widget_jobs, &QListWidget::currentItemChanged, this, &ToolPathPlannerWidget::onJobSelectionChanged);
  connect(ui_->push_button_load_selected_job, &QPushButton::clicked, this, &ToolPathPlannerWidget::loadSelectedJob);
  connect(ui_->push_button_save_job, &QPushButton::clicked, this, &ToolPathPlannerWidget::saveJob);

  // Signals & Slots for the Buttons on database management page
  connect(ui_->push_button_show_part, &QPushButton::clicked, this, &ToolPathPlannerWidget::showPartFromDatabase);
  connect(ui_->push_button_suppress_part, &QPushButton::clicked, this, &ToolPathPlannerWidget::deletePart);
  connect(ui_->push_button_suppress_job, &QPushButton::clicked, this, &ToolPathPlannerWidget::deleteJob);
  connect(ui_->push_button_refresh_parts, &QPushButton::clicked, this, &ToolPathPlannerWidget::refresh);
  connect(ui_->push_button_refresh_jobs, &QPushButton::clicked, this, &ToolPathPlannerWidget::refresh);

  // Add a publisher for the mesh marker
  pub_ = nh_.advertise<visualization_msgs::Marker>(MESH_MARKER_TOPIC, 1, true);

  // Set up the Database views in the third page
  std::string query = "`suppressed`!=\"1\"";
  model_parts_ = new QSqlTableModel(this, database_.getDatabase());
  model_parts_->setTable(QString::fromStdString(opp_db::PARTS_TABLE_NAME));
  model_parts_->setEditStrategy(QSqlTableModel::OnManualSubmit);
  model_parts_->select();
  model_parts_->setFilter(QString::fromStdString(query));
  model_parts_->removeColumn(model_parts_->columnCount() - 1);  // don't show the 'suppressed' column
  model_jobs_ = new QSqlTableModel(this, database_.getDatabase());
  model_jobs_->setTable(QString::fromStdString(opp_db::JOBS_TABLE_NAME));
  model_jobs_->setEditStrategy(QSqlTableModel::OnManualSubmit);
  model_jobs_->select();
  model_jobs_->setFilter(QString::fromStdString(query));
  model_jobs_->removeColumn(model_jobs_->columnCount() - 1);  // don't show the 'suppressed' column
  // Attach them to the views
  ui_->table_view_parts->setModel(model_parts_);
  ui_->table_view_parts->setEditTriggers(QTableView::NoEditTriggers);  // Make read only
  ui_->table_view_parts->show();
  ui_->table_view_jobs->setModel(model_jobs_);
  ui_->table_view_jobs->setEditTriggers(QTableView::NoEditTriggers);  // Make read only
  ui_->table_view_jobs->show();
}

void ToolPathPlannerWidget::setVisualizationFrame(const QString& text)
{
  marker_frame_ = text.toStdString();
  touch_point_editor_->setMarkerFrame(marker_frame_);
  verification_point_editor_->setMarkerFrame(marker_frame_);
  tool_path_editor_->setMarkerFrame(marker_frame_);
}

// Parts Page
void ToolPathPlannerWidget::browseForMeshResource()
{
  QString filename = QFileDialog::getOpenFileName(this, "Load Model", "", "Mesh Files (*.stl *.ply *.obj)");
  if (filename.isEmpty())
  {
    ROS_WARN_STREAM(__func__ << ": Empty filename");
    return;
  }

  ui_->line_edit_model_filename->setText(filename);
  loadMeshFromResource();
  return;
}

void ToolPathPlannerWidget::loadMeshFromResource()
{
  // Get the filename and package of the model
  std::string filename = ui_->line_edit_model_filename->text().toStdString();
  if (filename.empty())
  {
    QMessageBox::warning(this, "Input Error", "Model filename or package name not specified");
    return;
  }

  // Construct the mesh resource name using the package and filename
  std::vector<std::string> file_extensions = { ".stl", ".ply", ".obj" };

  mesh_resource_.clear();
  for (const std::string& ext : file_extensions)
  {
    std::regex rgx(".*" + ext + "$");
    std::smatch match;
    if (std::regex_search(filename, match, rgx))
    {
      mesh_resource_ = "file://" + filename;
      break;
    }
  }

  if (mesh_resource_.empty())
  {
    std::string message = "Invalid mesh resource file extension. Acceptable inputs are: ";
    for (const std::string& ext : file_extensions)
      message += ext + " ";

    QMessageBox::warning(this, "Input Error", QString(message.c_str()));
    return;
  }
  ROS_INFO_STREAM("Attempting to load mesh from resource: '" << mesh_resource_ << "'");

  if (!loadMesh())
    return;
}

void ToolPathPlannerWidget::loadModelsFromDatabase()
{
  // Create the variables the database interface will fill
  std::map<unsigned int, opp_msgs::Part> parts_map;
  std::string error_msg;

  // Retrieve part info from the database
  if (!database_.getAllPartsFromDatabase(error_msg, parts_map))
  {
    // If the function failed, create a warning pop-up box.
    std::string message = "Failed to retrieve parts from database, experienced error: " + error_msg;
    QMessageBox::warning(this, "Database Communication Error", QString::fromStdString(message));
  }
  else
  {
    // If the function succeeded, clear existing part lists
    existing_parts_.clear();
    ui_->list_widget_parts->clear();

    // And then add each part to those lists, one at a time
    for (const std::pair<unsigned int, opp_msgs::Part> id_and_part : parts_map)
    {
      // internal list of parts
      existing_parts_.push_back(id_and_part.second);

      // Gui display listing parts to user
      QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(id_and_part.second.name));
      item->setData(Qt::ItemDataRole::UserRole, QVariant(QString::fromStdString(id_and_part.second.description)));
      ui_->list_widget_parts->addItem(item);
    }
  }
  return;
}

void ToolPathPlannerWidget::onModelSelectionChanged(QListWidgetItem* current, QListWidgetItem*)
{
  // Change the description display
  if (current != nullptr)
  {
    ui_->text_edit_part_description->setText(current->data(Qt::ItemDataRole::UserRole).toString());
  }
}

void ToolPathPlannerWidget::loadSelectedModel()
{
  int row = ui_->list_widget_parts->currentRow();
  if (row >= 0 && row < static_cast<int>(existing_parts_.size()))
  {
    // Get the selected part information
    const opp_msgs::Part& part = existing_parts_[static_cast<std::size_t>(row)];
    generated_model_id_ = part.id;
    mesh_resource_ = part.mesh_resource;

    if (!loadMesh())
      return;

    // Update the UI and widgets with all part information
    ui_->line_edit_model_name->setText(QString::fromStdString(part.name));
    ui_->plain_text_edit_model_description->setPlainText(QString::fromStdString(part.description));

    touch_point_editor_->setPoints(part.touch_points);
    verification_point_editor_->setPoints(part.verification_points);

    setJobTabsEnabled(false, true);
    loadJobsFromDatabase();
  }
  else
  {
    QMessageBox::warning(this, "Input Error", "Make a selection in the parts list");
  }
}

void ToolPathPlannerWidget::saveModel()
{
  // Verify that the user intended to save the part
  QMessageBox::StandardButton button =
      QMessageBox::question(this, "Save Part to Database", "Proceed with adding the defined part to the database?");
  if (button == QMessageBox::StandardButton::No)
    return;

  // Get the relevant model information to save and verify it exists
  std::string model_name = ui_->line_edit_model_name->text().toStdString();
  std::string model_description = ui_->plain_text_edit_model_description->toPlainText().toStdString();
  if (model_name.empty() || model_description.empty())
  {
    QMessageBox::warning(this, "Input Error", "Model ID or description field(s) is empty");
    return;
  }

  // Get the touch points and verification points, and make sure there are
  // at least 3 of each.  (This requirement could probably be relaxed.)
  using TouchPointMap = std::map<std::string, opp_msgs::TouchPoint>;
  TouchPointMap touch_points = touch_point_editor_->getPoints();
  TouchPointMap verification_points = verification_point_editor_->getPoints();
  if (touch_points.size() < MIN_TOUCH_POINTS || verification_points.size() < MIN_VERIFICATION_POINTS)
  {
    QMessageBox::warning(this,
                         "Invalid Model Definition",
                         ("Ensure at least " + std::to_string(MIN_TOUCH_POINTS) + " touch points and " +
                          std::to_string(MIN_VERIFICATION_POINTS) + " verification points have been defined")
                             .c_str());
    return;
  }

  // Fill out the part struct with input information
  opp_msgs::Part part;
  part.name = model_name;
  part.description = model_description;
  part.mesh_resource = mesh_resource_;

  // Copy the touch points
  part.touch_points.reserve(touch_points.size());
  for (const std::pair<const std::string, opp_msgs::TouchPoint>& pair : touch_points)
  {
    part.touch_points.push_back(pair.second);
  }

  // Copy the verification points
  part.verification_points.reserve(verification_points.size());
  for (const std::pair<const std::string, opp_msgs::TouchPoint>& pair : verification_points)
  {
    part.verification_points.push_back(pair.second);
  }

  // Save the model to the database
  std::string error_msg;
  long int key = database_.addPartToDatabase(part, error_msg);
  if (key < 0)
  {
    // If the function failed, warn the user.
    std::string message = "Failed to add part to database, received error: " + error_msg;
    QMessageBox::warning(this, "Database Communication Error", QString(message.c_str()));
  }
  else
  {
    // Save the auto generated model id
    generated_model_id_ = key;

    // If the save is successful, allow the user to add job data for the part
    setJobTabsEnabled(true, true);

    // Make sure the database lists are updated
    refresh();
    loadModelsFromDatabase();
    loadJobsFromDatabase();
  }
  return;
}

// Jobs Page
void ToolPathPlannerWidget::newJob()
{
  std::vector<opp_msgs::ToolPath> empty;
  tool_path_editor_->addToolPathData(empty);
  setJobTabsEnabled(true, true);
  return;
}

void ToolPathPlannerWidget::loadJobsFromDatabase()
{
  std::map<unsigned int, opp_msgs::Job> jobs_map;
  std::string message;
  if (!database_.getAllJobsFromDatabase(generated_model_id_, message, jobs_map))
  {
    QMessageBox::warning(this, "Database Error", "Could not load any jobs for this part");
    return;
  }
  existing_jobs_.clear();
  ui_->list_widget_jobs->clear();
  for (std::pair<unsigned int, opp_msgs::Job> job_with_id : jobs_map)
  {
    QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(job_with_id.second.name));
    item->setData(Qt::ItemDataRole::UserRole, QVariant(QString::fromStdString(job_with_id.second.description)));
    ui_->list_widget_jobs->addItem(item);
    existing_jobs_.push_back(job_with_id.second);
  }

  return;
}

void ToolPathPlannerWidget::onJobSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous)
{
  (void)previous;

  // Change the description display
  if (current != nullptr)
  {
    ui_->text_edit_jobs->setText(current->data(Qt::ItemDataRole::UserRole).toString());
  }
  else
  {
    ui_->text_edit_jobs->setText(QString::fromStdString(std::string("")));
  }
  return;
}

void ToolPathPlannerWidget::loadSelectedJob()
{
  int row = ui_->list_widget_jobs->currentRow();
  if (row >= 0 && row < static_cast<int>(existing_jobs_.size()))
  {
    // Get the selected part information
    const opp_msgs::Job& job = existing_jobs_[static_cast<std::size_t>(row)];

    // Update the UI and widgets with all part information
    ui_->line_edit_job_name->setText(QString::fromStdString(job.name));
    ui_->plain_text_edit_job_description->setPlainText(QString::fromStdString(job.description));

    // Add the old paths to the tool path editor
    tool_path_editor_->addToolPathData(job.paths);

    setJobTabsEnabled(true, true);
  }
  else
  {
    QMessageBox::warning(this, "Input Error", "Make a selection in the jobs list");
  }
  return;
}

void ToolPathPlannerWidget::saveJob()
{
  // Verify that the user intended to press this button
  QMessageBox::StandardButton button =
      QMessageBox::question(this, "Save Job to Database", "Proceed with adding the defined job to the database?");
  if (button == QMessageBox::StandardButton::No)
  {
    return;
  }

  // Get the relevant job information to save, and ensure it is non-empty
  std::string job_name = ui_->line_edit_job_name->text().toStdString();
  std::string job_description = ui_->plain_text_edit_job_description->toPlainText().toStdString();
  if (job_name.empty() || job_description.empty())
  {
    QMessageBox::warning(this, "Input Error", "Job ID or description is invalid");
    return;
  }

  // Construct a job object
  opp_msgs::Job job;
  job.name = job_name;
  job.description = job_description;
  job.part_id = generated_model_id_;
  job.header.stamp = ros::Time::now();

  // Get the tool paths and add them to the job
  ToolPathDataMap tool_paths = tool_path_editor_->getToolPathData();
  job.paths.reserve(tool_paths.size());
  for (const std::pair<const std::string, opp_msgs::ToolPath>& pair : tool_paths)
  {
    job.paths.push_back(pair.second);
  }

  // Save the job to the database
  std::string error_msg;
  long int key = database_.addJobToDatabase(job, error_msg);
  if (key < 0)
  {
    // If the function failed, warn the user.
    std::string message = "Failed to add job to database, received error: " + error_msg;
    QMessageBox::warning(this, "Database Communication Error", QString(message.c_str()));
  }
  else
  {
    // update the database views
    refresh();
    loadJobsFromDatabase();
  }

  return;
}

// Database Management Page
void ToolPathPlannerWidget::showPartFromDatabase()
{
  // Get the part id of the currently selected row
  QModelIndex index = ui_->table_view_parts->currentIndex();
  QSqlRecord part_record = model_parts_->record(index.row());
  QSqlField part_id_field = part_record.field("part_id");
  QVariant part_id_value = part_id_field.value();
  unsigned int part_id = static_cast<unsigned int>(part_id_value.toInt());

  // Get the part definition from the database
  std::string msg;
  opp_msgs::Part part;
  if (!database_.getPartFromDatabase(part_id, msg, part))
  {
    ROS_ERROR_STREAM("could not load selected part, had error: " << msg);
    return;
  }

  // Set class parameters using the loaded part
  generated_model_id_ = part.id;
  mesh_resource_ = part.mesh_resource;

  if (!loadMesh())
    return;

  // Update the UI and widgets with all part information
  ui_->line_edit_model_name->setText(QString::fromStdString(part.name));
  ui_->plain_text_edit_model_description->setPlainText(QString::fromStdString(part.description));

  touch_point_editor_->setPoints(part.touch_points);
  verification_point_editor_->setPoints(part.verification_points);

  setJobTabsEnabled(false, true);
  loadJobsFromDatabase();
}

void ToolPathPlannerWidget::deletePart()
{
  // Get the part id of the currently selected row
  QModelIndex index = ui_->table_view_parts->currentIndex();
  QSqlRecord part_record = model_parts_->record(index.row());
  QSqlField part_id_field = part_record.field("part_id");
  QVariant part_id_value = part_id_field.value();
  unsigned int part_id = static_cast<unsigned int>(part_id_value.toInt());

  // Suppress (but do not actually delete) the part
  std::string msg;
  if (!database_.suppressPart(part_id, msg))
  {
    ROS_ERROR_STREAM("could not suppress selected part, had error: " << msg);
  }

  // In case we have the to-be-suppressed part open, disable job creation.
  // They will have to save as a new part before creating a new job.
  if (part_id == generated_model_id_)
  {
    setModelTabsEnabled(false);
    setJobTabsEnabled(false, false);
  }

  // Refresh the various database displays to ensure no 'ghosts' remain.
  refresh();
  loadModelsFromDatabase();
  return;
}

void ToolPathPlannerWidget::deleteJob()
{
  // Get the id of the currently selected row
  QModelIndex index = ui_->table_view_jobs->currentIndex();
  QSqlRecord job_record = model_jobs_->record(index.row());
  QSqlField id_field = job_record.field("id");
  QVariant id_value = id_field.value();
  unsigned int id = static_cast<unsigned int>(id_value.toInt());

  // Suppress (but do not actually delete) the job
  std::string msg;
  if (!database_.suppressJob(id, msg))
  {
    ROS_ERROR_STREAM("could not suppress selected job, had error: " << msg);
  }

  // Refresh the displays to chase out any 'ghosts'
  refresh();
  return;
}

void ToolPathPlannerWidget::refresh()
{
  std::string query = "`suppressed`!=\"1\"";
  model_parts_->setFilter(QString::fromStdString(query));
  model_jobs_->setFilter(QString::fromStdString(query));

  model_parts_->select();
  model_jobs_->select();
}

// Private functions
void ToolPathPlannerWidget::clear()
{
  // Clear the data in the list editor widgets
  touch_point_editor_->clear();
  verification_point_editor_->clear();
  tool_path_editor_->clear();

  // Clear the text input data about the model
  ui_->line_edit_model_name->clear();
  ui_->plain_text_edit_model_description->clear();

  // Clear the text input data about the job
  ui_->line_edit_job_name->clear();
  ui_->plain_text_edit_job_description->clear();
}

bool ToolPathPlannerWidget::loadMesh()
{
  // Attempt to load this file into a shape_msgs/Mesh type
  shape_msgs::Mesh mesh;
  if (!utils::getMeshMsgFromResource(mesh_resource_, mesh))
  {
    std::string message = "Failed to load mesh from resource: '" + mesh_resource_ + "'";
    QMessageBox::warning(this, "Input Error", message.c_str());
    return false;
  }

  // Clear the touch point and tool path editors' data before continuing
  clear();

  // Initialize the tool path editor with the mesh
  tool_path_editor_->init(mesh);

  // Enable the models tabs but not the jobs tabs
  setModelTabsEnabled(true);
  setJobTabsEnabled(false, false);

  // Publish the mesh marker
  visualization_msgs::Marker mesh_marker =
      utils::createMeshMarker(0, "mesh", Eigen::Isometry3d::Identity(), marker_frame_, mesh_resource_);

  pub_.publish(mesh_marker);

  return true;
}

void ToolPathPlannerWidget::setModelTabsEnabled(bool enabled)
{
  for (int i = 1; i < ui_->tool_box_model_editor->count(); ++i)
  {
    ui_->tool_box_model_editor->setItemEnabled(i, enabled);
  }

  ui_->frame_define_touch_off_points->setEnabled(enabled);
  ui_->frame_define_verification_points->setEnabled(enabled);
}

void ToolPathPlannerWidget::setJobTabsEnabled(bool enabled, bool first_enabled)
{
  for (int i = 1; i < ui_->tool_box_job_editor->count(); ++i)
  {
    ui_->tool_box_job_editor->setItemEnabled(i, enabled);
  }
  if (ui_->tool_box_job_editor->count() > 0)
  {
    ui_->tool_box_job_editor->setItemEnabled(0, first_enabled);
    ui_->push_button_new_job->setEnabled(first_enabled);
    ui_->push_button_load_jobs_from_database->setEnabled(first_enabled);
    ui_->push_button_load_selected_job->setEnabled(first_enabled);
  }
  ui_->frame_define_toolpaths->setEnabled(enabled);
}

}  // namespace opp_gui
