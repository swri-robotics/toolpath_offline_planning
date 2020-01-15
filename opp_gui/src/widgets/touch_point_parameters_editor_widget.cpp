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

#include "opp_gui/widgets/touch_point_parameters_editor_widget.h"
#include "ui_touch_point_parameters_editor.h"

const std::string CLICKED_POINT_TOPIC = "/selection_point";

namespace opp_gui
{
TouchPointParametersEditorWidget::TouchPointParametersEditorWidget(QWidget* parent)
  : QWidget(parent), accept_mouse_input_(false)
{
  ui_ = new Ui::TouchPointParametersEditor();
  ui_->setupUi(this);

  sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      CLICKED_POINT_TOPIC, 1, &TouchPointParametersEditorWidget::callback, this);

  // Make the position line edits only accept numbers
  ui_->double_spin_box_x->setRange(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  ui_->double_spin_box_y->setRange(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  ui_->double_spin_box_z->setRange(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

  ui_->double_spin_box_qw->setRange(-1.0, 1.0);
  ui_->double_spin_box_qx->setRange(-1.0, 1.0);
  ui_->double_spin_box_qy->setRange(-1.0, 1.0);
  ui_->double_spin_box_qz->setRange(-1.0, 1.0);

  // The threshold on touchpoints should be a positive number
  ui_->double_spin_box_threshold->setRange(0.0, std::numeric_limits<double>::max());

  // Connect signals and slots
  connect(ui_->push_button_select_with_mouse,
          &QPushButton::clicked,
          this,
          &TouchPointParametersEditorWidget::onSelectWithMouse);
  connect(ui_->push_button_update, &QPushButton::clicked, this, &TouchPointParametersEditorWidget::dataChanged);
}

void TouchPointParametersEditorWidget::onSelectWithMouse()
{
  if (!accept_mouse_input_)
  {
    // Keep the push button flat until we have received a message back
    ui_->push_button_select_with_mouse->setStyleSheet("background-color: red");
    ui_->push_button_select_with_mouse->setFlat(true);

    // Update the flag for accepting mouse input
    accept_mouse_input_ = true;
  }
  else
  {
    // Toggle this functionality back off
    ui_->push_button_select_with_mouse->setStyleSheet("");
    ui_->push_button_select_with_mouse->setFlat(false);

    accept_mouse_input_ = false;
  }
}

void TouchPointParametersEditorWidget::callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (accept_mouse_input_)
  {
    // Reset the button visualization
    ui_->push_button_select_with_mouse->setStyleSheet("");
    ui_->push_button_select_with_mouse->setFlat(false);

    // Set the pose
    setPose(msg->pose);

    // Reset the flag for accepting mouse input
    accept_mouse_input_ = false;

    emit dataChanged();
  }
}

geometry_msgs::Pose TouchPointParametersEditorWidget::getPose() const
{
  geometry_msgs::Pose pose;

  // Get the data from the line edit fields
  pose.position.x = ui_->double_spin_box_x->text().toDouble();
  pose.position.y = ui_->double_spin_box_y->text().toDouble();
  pose.position.z = ui_->double_spin_box_z->text().toDouble();

  pose.orientation.w = ui_->double_spin_box_qw->text().toDouble();
  pose.orientation.x = ui_->double_spin_box_qx->text().toDouble();
  pose.orientation.y = ui_->double_spin_box_qy->text().toDouble();
  pose.orientation.z = ui_->double_spin_box_qz->text().toDouble();

  return pose;
}

void TouchPointParametersEditorWidget::setPose(const geometry_msgs::Pose& pose)
{
  // Save the data from the incoming message to the line edit fields
  ui_->double_spin_box_x->setValue(pose.position.x);
  ui_->double_spin_box_y->setValue(pose.position.y);
  ui_->double_spin_box_z->setValue(pose.position.z);

  ui_->double_spin_box_qw->setValue(pose.orientation.w);
  ui_->double_spin_box_qx->setValue(pose.orientation.x);
  ui_->double_spin_box_qy->setValue(pose.orientation.y);
  ui_->double_spin_box_qz->setValue(pose.orientation.z);
}

void TouchPointParametersEditorWidget::setTouchPoint(const opp_msgs::TouchPoint& tp)
{
  ui_->text_edit_description->setText(QString::fromStdString(tp.description));
  ui_->double_spin_box_threshold->setValue(tp.threshold);
  setPose(tp.transform.pose);
}

opp_msgs::TouchPoint TouchPointParametersEditorWidget::getTouchPoint() const
{
  opp_msgs::TouchPoint out;
  out.description = ui_->text_edit_description->toPlainText().toStdString();
  out.threshold = ui_->double_spin_box_threshold->value();
  out.transform.pose = getPose();
  return out;
}

}  // namespace opp_gui
