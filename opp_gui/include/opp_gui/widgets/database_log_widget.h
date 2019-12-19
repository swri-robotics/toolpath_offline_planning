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

#ifndef DATABASE_LOG_WIDGET_H
#define DATABASE_LOG_WIDGET_H

#include <QString>
#include <QTimer>
#include <QWidget>
#include <opp_application/application/application_context_base.h>
#include <ros/service.h>
#include <map>
#include <opp_msgs/Job.h> // TODO add the Log service to msgs, and change this to Log.h

namespace Ui {
  class DatabaseLog;
}

class QSqlTableModel;

namespace opp_gui
{

/**
 * @brief Widget that displays all recent jobs performed
 */
class DatabaseLogWidget : public QWidget
{
  Q_OBJECT

 public:

  DatabaseLogWidget(opp_application::ApplicationContextBasePtr app,
		    QWidget *parent = nullptr);
  
  ~DatabaseLogWidget();
    
protected Q_SLOTS:
  void refresh();
  void filter(const QString& text);
      
protected:
      
  Ui::DatabaseLog* ui_;
  ros::NodeHandle nh_;
  QTimer refresh_timer_;
  QSqlTableModel *model_;

  opp_application::ApplicationContextBasePtr app_;
};
} // end of namespace opp_gui

#endif // DATABASE_LOG_WIDGET_H
