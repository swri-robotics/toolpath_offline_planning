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

#include <QMessageBox>

#include <opp_gui/widgets/database_selection_widget.h>
#include <opp_application/application/application_context_base.h>
#include <opp_msgs/GetAllJobsFromDatabase.h>
#include <opp_msgs/GetAllPartsFromDatabase.h>
#include "ui_database_browser.h"

namespace opp_gui
{

// Constructor that uses default initial-transform value
PartSelectorWidget::PartSelectorWidget(opp_application::ApplicationContextBasePtr app,
                                       QWidget* parent)
  : DatabaseBrowserWidget(parent)
  , app_(app),
    selected_part_wcd_("")
{
  // Initialize the initial_part_location transform to identity
  initial_part_location_.pose.orientation.w = 1.0;

  // Connect the button
  ui_->push_button_select->setText("Add Part");
  ui_->tree_widget_db_entries->setRootIsDecorated(false);
  ui_->WorkCD_lineEdit->setReadOnly(false);
  connect(ui_->push_button_update, &QPushButton::clicked, this, &PartSelectorWidget::onLoadDatabase);
  connect(ui_->push_button_select, &QPushButton::clicked, this, &PartSelectorWidget::onSelectPart);

  // Connect the lineEdit
  connect(ui_->WorkCD_lineEdit, &QLineEdit::editingFinished, this, &PartSelectorWidget::onEnterWID);
  ui_->WCD_Label->setText("Enter WCD");
}

// Constructor that specifies the initial transform
PartSelectorWidget::PartSelectorWidget(opp_application::ApplicationContextBasePtr app,
                                       const geometry_msgs::PoseStamped initial_part_location,
                                       QWidget* parent)
  : DatabaseBrowserWidget(parent)
  , app_(app)
  , initial_part_location_(initial_part_location),
    selected_part_wcd_("")
{
  // Connect the button
  ui_->push_button_select->setText("Add Part");
  ui_->tree_widget_db_entries->setRootIsDecorated(false);
  connect(ui_->push_button_update, &QPushButton::clicked, this, &PartSelectorWidget::onLoadDatabase);
  connect(ui_->push_button_select, &QPushButton::clicked, this, &PartSelectorWidget::onSelectPart);

  // Connect the lineEdit
  connect(ui_->WorkCD_lineEdit, &QLineEdit::editingFinished, this, &PartSelectorWidget::onEnterWID);
}

bool PartSelectorWidget::onSelectPart()
{
  selected_part_wcd_ = ui_->WorkCD_lineEdit->text().toStdString();
  if(selected_part_wcd_ == ""){
    QMessageBox MB;
    MB.setText("No Work ID entered");
    MB.exec();
    return false;
  }
  QTreeWidgetItem* current = ui_->tree_widget_db_entries->currentItem();
  if(current)
  {
    // Define a lambda for attempting to set the current part
    auto try_set_part = [this](QVariant& part_data)->bool
    {
      auto part_it = part_map_.find(qvariant_cast<uint32_t>(part_data));

      if(part_it != part_map_.end())
      {
        // Set the active part
        opp_application::TaskResponse res = app_->addPart(part_it->second, initial_part_location_, selected_part_wcd_);
        if(res.success)
        {
          emit okToProceed(true);
          ui_->WorkCD_lineEdit->setText("");
          return true;
        }
        else
        {
          QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
        }
      }

      return false;
    };

    // Get the data from the current selection
    QVariant part_data = current->data(0, PART_ROLE);

    // Try to set the active part in the application
    if(try_set_part(part_data))
    {
      ui_->WorkCD_lineEdit->setText("");
      return true;
    }
    else
    {
      /* Maybe the user has selected a job; get the current selection's parent (which may be a part)
       * and try to set it as the active part */
      QTreeWidgetItem* part = current->parent();

      // Make sure the current selection has a parent
      if(part)
      {
        part_data = part->data(0, PART_ROLE);
        if(try_set_part(part_data))
        {
          ui_->WorkCD_lineEdit->setText("");
          return true;
        }
      }
      else
      {
        QMessageBox::warning(this, "Widget Error", "The current selection does not have a parent");
      }
    }
  }
  else
  {
    QMessageBox::warning(this, "Input Error", "Please select a part in the tree");
  }

  emit okToProceed(false);
}

void PartSelectorWidget::onEnterWID()
{
  // TODO add something here if necessary or delete. This function was created for debugging, but no longer used  
}

void PartSelectorWidget::onLoadDatabase()
{
  ui_->tree_widget_db_entries->clear();
  part_map_.clear();
  job_map_.clear();

  opp_msgs::GetAllPartsFromDatabase srv;
  if(!get_all_parts_client_.call(srv))
  {
    std::string message = "Failed to call '" + get_all_parts_client_.getService() + "' service";
    QMessageBox::warning(this, "ROS Communication Error", QString::fromStdString(message));
    return;
  }
  else if(!srv.response.success)
  {
    QMessageBox::warning(this, "Database Error", QString::fromStdString(srv.response.message));
    return;
  }

  // Error message
  std::string job_load_error;
  // Iterate over the parts and add the jobs associated with each one
  ui_->tree_widget_db_entries->setSortingEnabled(false);
  for(const opp_msgs::Part& part : srv.response.parts)
  {
    // Add the current part to the persistent part map
    part_map_.emplace(part.id, part);

    // Add a QTreeWidgetItem for the part
    QTreeWidgetItem* q_part = new QTreeWidgetItem();
    q_part->setText(0, QString::fromStdString(part.name));
    q_part->setData(0, PART_ROLE, QVariant(part.id));

    // Call the service to get the jobs for this part from the database
    opp_msgs::GetAllJobsFromDatabase srv;
    srv.request.part_id = part.id;

    if(!get_all_jobs_client_.call(srv))
    {
      job_load_error += part.name + ": Failed to call '" + get_all_jobs_client_.getService() + "' service\n";
    }
    else if(!srv.response.success)
    {
      job_load_error += part.name + "\n";
    }
    else
    {
      // Iterate over all of the jobs associated with this part and add them as children to the tree visualization
      for(const opp_msgs::Job& job : srv.response.jobs)
      {
        job_map_.emplace(job.id, job);

        QTreeWidgetItem* q_job = new QTreeWidgetItem();
        q_job->setText(0, QString::fromStdString(job.name));
        q_job->setData(0, JOB_ROLE, QVariant(job.id));

        q_part->addChild(q_job);
      }
    }

    // Add the part and its children jobs to the tree widget
    ui_->tree_widget_db_entries->addTopLevelItem(q_part);
  }
  ui_->tree_widget_db_entries->setSortingEnabled(true);

  // Enable the panes
  ui_->tree_widget_db_entries->setEnabled(true);
  ui_->text_edit_description->setEnabled(true);
  ui_->push_button_select->setEnabled(true);

  if(!job_load_error.empty())
  {
    std::string message = "Failed to load jobs for the following parts:\n\n" + job_load_error;
    QMessageBox::warning(this, "Database Error", QString::fromStdString(message));
  }

  emit okToProceed(false);
}


JobSelectorWidget::JobSelectorWidget(opp_application::ApplicationContextBasePtr app,
                                     QWidget* parent)
  : DatabaseBrowserWidget(parent)
  , app_(app)
{
  ui_->push_button_select->setText("Select Job");
  ui_->WorkCD_lineEdit->setText(app_->active_part_->second.work_control_document.c_str());
  ui_->WCD_Label->setText("Active WCD");
  ui_->WorkCD_lineEdit->setReadOnly(true);
  connect(ui_->push_button_update, &QPushButton::clicked, this, &JobSelectorWidget::onLoadDatabase);
  connect(ui_->push_button_select, &QPushButton::clicked, this, &JobSelectorWidget::onSelectJob);
}

void JobSelectorWidget::onSelectJob()
{
  QTreeWidgetItem* current = ui_->tree_widget_db_entries->currentItem();
  if(current)
  {
    QVariant job_data = current->data(0, JOB_ROLE);
    auto job_it = job_map_.find(qvariant_cast<uint32_t>(job_data));

    if(job_it != job_map_.end())
    {
      // Set the active job
      ui_->WorkCD_lineEdit->setText(app_->active_part_->second.work_control_document.c_str());
      opp_application::TaskResponse res = app_->setActiveJob(job_it->second);
      if(res.success)
      {
        emit okToProceed(true);
        return;
      }
      else
      {
        QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
      }
    }
    else
    {
      QMessageBox::warning(this, "Input Error", "Selection is not a known job");
    }
  }
  else
  {
    QMessageBox::warning(this, "Input Error", "Please select a job in the tree");
  }

  emit okToProceed(false);
}

void JobSelectorWidget::onPartSelect()
{
  int topCount_ = ui_->tree_widget_db_entries->topLevelItemCount();
  for (int i = 0; i < topCount_; i++)
  {
    QTreeWidgetItem* current = ui_->tree_widget_db_entries->topLevelItem(i);
    if(current)
    {
      QVariant part_data = current->data(0, PART_ROLE);
      auto part_it = part_map_.find(qvariant_cast<uint32_t>(part_data));

      if(app_->active_part_->second.part.id == part_it->second.id)
      {
        current->setHidden(false);
        current->setExpanded(true);
      }
      else
      {
        current->setHidden(true);
      }
    }
  }
}

void JobSelectorWidget::onLoadDatabase()
{
  ui_->tree_widget_db_entries->clear();
  part_map_.clear();
  job_map_.clear();

  opp_msgs::GetAllPartsFromDatabase srv;
  if(!get_all_parts_client_.call(srv))
  {
    std::string message = "Failed to call '" + get_all_parts_client_.getService() + "' service";
    QMessageBox::warning(this, "ROS Communication Error", QString::fromStdString(message));
    return;
  }
  else if(!srv.response.success)
  {
    QMessageBox::warning(this, "Database Error", QString::fromStdString(srv.response.message));
    return;
  }

  // Error message
  std::string job_load_error;
  // Iterate over the parts and add the jobs associated with each one
  ui_->tree_widget_db_entries->setSortingEnabled(false);
  for(const opp_msgs::Part& part : srv.response.parts)
  {
    // Add the current part to the persistent part map
    part_map_.emplace(part.id, part);

    // Add a QTreeWidgetItem for the part
    QTreeWidgetItem* q_part = new QTreeWidgetItem();
    q_part->setText(0, QString::fromStdString(part.name));
    q_part->setData(0, PART_ROLE, QVariant(part.id));

    // Call the service to get the jobs for this part from the database
    opp_msgs::GetAllJobsFromDatabase srv;
    srv.request.part_id = part.id;

    if(!get_all_jobs_client_.call(srv))
    {
      job_load_error += part.name + ": Failed to call '" + get_all_jobs_client_.getService() + "' service\n";
    }
    else if(!srv.response.success)
    {
      job_load_error += part.name + "\n";
    }
    else
    {
      // Iterate over all of the jobs associated with this part and add them as children to the tree visualization
      for(const opp_msgs::Job& job : srv.response.jobs)
      {
        job_map_.emplace(job.id, job);

        QTreeWidgetItem* q_job = new QTreeWidgetItem();
        q_job->setText(0, QString::fromStdString(job.name));
        q_job->setData(0, JOB_ROLE, QVariant(job.id));

        q_part->addChild(q_job);
      }
    }
    // Add the part and its children jobs to the tree widget
    ui_->tree_widget_db_entries->addTopLevelItem(q_part);
    QTreeWidgetItem* current = ui_->tree_widget_db_entries->topLevelItem(ui_->tree_widget_db_entries->topLevelItemCount()-1);
    if(part.id != app_->active_part_->second.part.id)
    {
      current->setHidden(true);
    }
    else
    {
      current->setExpanded(true);
      current->setHidden(false);
    }
  }
  ui_->tree_widget_db_entries->setSortingEnabled(true);

  // Enable the panes
  ui_->tree_widget_db_entries->setEnabled(true);
  ui_->text_edit_description->setEnabled(true);
  ui_->push_button_select->setEnabled(true);

  if(!job_load_error.empty())
  {
    std::string message = "Failed to load jobs for the following parts:\n\n" + job_load_error;
    QMessageBox::warning(this, "Database Error", QString::fromStdString(message));
  }

  emit okToProceed(false);
}
} // namespace opp_gui
