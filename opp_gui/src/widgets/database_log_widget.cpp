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

#include "opp_gui/widgets/database_log_widget.h"
#include "ui_database_log.h"
#include <QtSql/QSqlTableModel>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>

namespace opp_gui
{
  
DatabaseLogWidget::DatabaseLogWidget(opp_application::ApplicationContextBasePtr app,
				     QWidget *parent) :
  QWidget(parent)
  , app_(app)
{
  ui_ = new Ui::DatabaseLog();
  ui_->setupUi(this);

  model_ = new QSqlTableModel(this, app_->database_.getDatabase());
  model_->setTable(QString::fromStdString(opp_db::JOB_LOG_TABLE_NAME));
  model_->setEditStrategy(QSqlTableModel::OnManualSubmit);
  model_->select();
  model_->removeColumn(0); // don't show the ID

  connect(&refresh_timer_, SIGNAL(timeout()), this, SLOT(refresh()));
  refresh_timer_.setInterval(10000);
  refresh_timer_.start();

  connect(ui_->line_edit_wcd_entry, &QLineEdit::textChanged, this, &DatabaseLogWidget::filter);

  // Attach it to the view
  ui_->log_table_view->setModel(model_);
  ui_->log_table_view->setEditTriggers(QTableView::NoEditTriggers); // Make read only
  ui_->log_table_view->show();
}

DatabaseLogWidget::~DatabaseLogWidget()
{
  delete ui_;
}

void DatabaseLogWidget::refresh()
{
  model_->select();
  refresh_timer_.start();
}

void DatabaseLogWidget::filter(const QString& text)
{
  std::string query = "`work control document number` LIKE \"" + text.toStdString() + "%\"";
  model_->setFilter(QString::fromStdString(query));
}

}// end namespace opp_gui
