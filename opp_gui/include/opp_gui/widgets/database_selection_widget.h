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

#ifndef OPP_GUI_WIDGETS_DATABASE_SELECTION_WIDGET_H
#define OPP_GUI_WIDGETS_DATABASE_SELECTION_WIDGET_H

#include <geometry_msgs/Transform.h>

#include <opp_gui/widgets/database_browser_widget.h>

namespace opp_application
{
class ApplicationContextBase;
typedef typename std::shared_ptr<ApplicationContextBase> ApplicationContextBasePtr;
typedef typename boost::function<void(bool)> NotifyCbType;
}

namespace opp_gui
{

/**
 * @brief Inherits the DatabaseBrowserWidget class and uses its functionality to set the active part
 * in the application from the parts available in the database
 */
class PartSelectorWidget : public DatabaseBrowserWidget
{
Q_OBJECT
public:

  /**
   * @brief PartSelectorWidget - Constructor that sets the initial part
   * transform to identity
   * @param app - input - the application context object being used to
   * connect to the underlying ROS system
   * @param parent - input - the QT object which is this widget's parent
   */
  PartSelectorWidget(opp_application::ApplicationContextBasePtr app,
                     QWidget* parent = nullptr);

  /**
   * @brief PartSelectorWidget - Constructor that sets the initial part
   * transform to a calling-code-defined transform
   * @param app - input - the application context object being used to
   * connect to the underlying ROS system
   * @param initial_part_location - input - the position at which parts
   * should first appear when added
   * @param parent - input - the QT object which is this widget's parent
   */
  PartSelectorWidget(opp_application::ApplicationContextBasePtr app,
                     const geometry_msgs::PoseStamped initial_part_location,
                     QWidget* parent = nullptr);

 public:
  /** &brief the currently selected part's work control document number **/
  std::string selected_part_wcd_;

protected Q_SLOTS:

  /** @brief a function that is called whenever the 'select part' button
   * in this widget is clicked. It adds the part to the list of parts in
   * the application context object.
   */
  bool onSelectPart();
  /** @brief a function that is called whenever the WID is entered
   *  It enables the "Select" button so a part is added
   */
  void onEnterWID();
  /** @brief loads all parts/jobs into the part selector widget
   */
  void onLoadDatabase();

private:

  /** @brief the application context ojbect used to connect to the system */
  opp_application::ApplicationContextBasePtr app_;

  /** @brief the position at which parts should first appear */
  geometry_msgs::PoseStamped initial_part_location_;

};

/**
 * @brief Inherits the DatabaseBrowserWidget class and uses its functionality to set the active job
 * in the application from the jobs available in the database
 */
class JobSelectorWidget : public DatabaseBrowserWidget
{
Q_OBJECT
public:

  JobSelectorWidget(opp_application::ApplicationContextBasePtr app,
                    QWidget* parent = nullptr);
  /** @brief a function that is called when the active part is chagned
   *  displays the active parts jobs and hides the jobs for all other parts
   */
  void onPartSelect();
protected Q_SLOTS:

  /** @brief a function that is called whenever the select job button is pressed
   */
  void onSelectJob();
  /** @brief loads all parts/jobs into the part selector widget.
   *  Only shows the entry for the active job
   */
  void onLoadDatabase();


private:

  opp_application::ApplicationContextBasePtr app_;
};

} // namespace opp_gui

#endif // OPP_GUI_WIDGETS_DATABASE_SELECTION_WIDGET_H
