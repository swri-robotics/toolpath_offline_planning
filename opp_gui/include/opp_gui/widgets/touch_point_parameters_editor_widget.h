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

#ifndef OPP_GUI_WIDGETS_TOUCH_POINT_PARAMETERS_EDITOR_H
#define OPP_GUI_WIDGETS_TOUCH_POINT_PARAMETERS_EDITOR_H

#include <atomic>

#include <QWidget>

#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <opp_msgs/TouchPoint.h>

namespace Ui
{
class TouchPointParametersEditor;
}

namespace opp_gui
{
/**
 * @brief Widget for editing the parameters associated with a touch-off point
 */
class TouchPointParametersEditorWidget : public QWidget
{
  Q_OBJECT
public:
  TouchPointParametersEditorWidget(QWidget* parent = nullptr);

  opp_msgs::TouchPoint getTouchPoint() const;

  void setTouchPoint(const opp_msgs::TouchPoint& tp);

Q_SIGNALS:

  /**
   * @brief Signal emitted when data has changed in one of the editable UI fields
   */
  void dataChanged();

protected Q_SLOTS:

  /**
   * @brief Callback that allows the widget to listen for pose data from an external topic
   * (i.e. from RViz) with which to set the touch point position and orientation fields
   */
  void onSelectWithMouse();

private:
  void setPose(const geometry_msgs::Pose& pose);

  geometry_msgs::Pose getPose() const;

  void callback(const geometry_msgs::PoseStampedConstPtr& msg);

  Ui::TouchPointParametersEditor* ui_;

  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  std::atomic<bool> accept_mouse_input_;
};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOUCH_POINT_PARAMETERS_EDITOR_H
