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
#include <opp_gui/widgets/database_browser_widget.h>
#include <opp_msgs/GetAllJobsFromDatabase.h>
#include <opp_msgs/GetAllPartsFromDatabase.h>
#include "ui_database_browser.h"

const static std::string GET_ALL_PARTS_SERVICE = "get_all_parts_from_db";
const static std::string GET_ALL_JOBS_SERVICE = "get_all_jobs_from_db";

namespace opp_gui
{

DatabaseBrowserWidget::DatabaseBrowserWidget(QWidget* parent)
  : QWidget(parent)
{
  ui_ = new Ui::DatabaseBrowser();
  ui_->setupUi(this);

  // Disable the jobs pane until a part selection has been made
  ui_->tree_widget_db_entries->setEnabled(false);
  ui_->text_edit_description->setEnabled(false);
  ui_->push_button_select->setEnabled(false);

  connect(ui_->tree_widget_db_entries, &QTreeWidget::itemClicked, this, &DatabaseBrowserWidget::onTreeItemSelected);

  get_all_parts_client_ = nh_.serviceClient<opp_msgs::GetAllPartsFromDatabase>(GET_ALL_PARTS_SERVICE);
  get_all_jobs_client_ = nh_.serviceClient<opp_msgs::GetAllJobsFromDatabase>(GET_ALL_JOBS_SERVICE);
}


void DatabaseBrowserWidget::onTreeItemSelected(QTreeWidgetItem* item)
{
  if(item)
  {
    QVariant part_data = item->data(0, PART_ROLE);
    auto part_it = part_map_.find(qvariant_cast<uint32_t>(part_data));

    if(part_it != part_map_.end())
    {
      ui_->text_edit_description->setText(QString::fromStdString(part_it->second.description));
    }

    QVariant job_data = item->data(0, JOB_ROLE);
    auto job_it = job_map_.find(qvariant_cast<uint32_t>(job_data));

    if(job_it != job_map_.end())
    {
      ui_->text_edit_description->setText(QString::fromStdString(job_it->second.description));
    }
  }

  emit okToProceed(false);
}

} // namespace opp_gui
