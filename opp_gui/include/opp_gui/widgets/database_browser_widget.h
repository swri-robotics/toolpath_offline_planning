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

#ifndef DATABASE_BROWSER_H
#define DATABASE_BROWSER_H

#include <QWidget>
#include <ros/service.h>
#include <opp_msgs/Part.h>
#include <opp_msgs/Job.h>
#include <map>

class QTreeWidgetItem;

namespace Ui
{
class DatabaseBrowser;
}

namespace opp_gui
{

/**
 * @brief Widget that displays all parts and jobs known to the database in a tree hierarchy form
 */
class DatabaseBrowserWidget : public QWidget
{
Q_OBJECT
public:

  DatabaseBrowserWidget(QWidget* parent = nullptr);

  virtual ~DatabaseBrowserWidget() = default;

Q_SIGNALS:

  /**
   * @brief Signal indicating whether or not the selection in the tree (part or job) has been set
   * active in the application. This signal can be used by dependent widgets (namely a higher level
   * application GUI) to determine whether they can proceed in their process
   * @param status
   */
  void okToProceed(bool status);

protected Q_SLOTS:

  void onTreeItemSelected(QTreeWidgetItem* item);

protected:

  Ui::DatabaseBrowser* ui_;

  ros::NodeHandle nh_;
  ros::ServiceClient get_all_parts_client_;
  ros::ServiceClient get_all_jobs_client_;

  std::map<uint32_t, opp_msgs::Part> part_map_;
  std::map<uint32_t, opp_msgs::Job> job_map_;

  const static int PART_ROLE = Qt::ItemDataRole::UserRole + 1;
  const static int JOB_ROLE = Qt::ItemDataRole::UserRole + 2;
};

} // namespace opp_gui

#endif // DATABASE_BROWSER_H
