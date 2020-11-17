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

#include "opp_database/opp_database_interface_cpp.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <QSqlError>
#include <QSqlQuery>
#include <QStringList>
#include <QVariant>

#include <ros/console.h>

#include <message_serialization/serialize.h>
#include <message_serialization/trajectory_msgs_yaml.h>
#include <message_serialization/geometry_msgs_yaml.h>
#include <message_serialization/std_msgs_yaml.h>
#include <message_serialization/sensor_msgs_yaml.h>
#include <message_serialization/shape_msgs_yaml.h>
#include <message_serialization/eigen_yaml.h>
#include <opp_msgs_serialization/opp_msgs_yaml.h>

namespace opp_db
{
ROSDatabaseInterface::ROSDatabaseInterface()
{
  database_ = QSqlDatabase::addDatabase("QMYSQL");
  database_.setHostName("localhost");
  database_.setDatabaseName("opp");
  database_.setUserName("ros-client");
  database_.setPassword("0000");
  if (!database_.open())
  {
    ROS_ERROR("Failed to connect to database");
  }
  else
  {
    createJobsTable();
    createPartsTable();
  }
  save_dir_ = std::string(std::getenv("HOME")) + "/.local/share/offline_generated_paths";
}

ROSDatabaseInterface::~ROSDatabaseInterface() { database_.close(); }

QSqlDatabase& ROSDatabaseInterface::getDatabase() { return database_; }

bool ROSDatabaseInterface::isConnected() const { return database_.isOpen(); }

long int ROSDatabaseInterface::addJobToDatabase(const opp_msgs::Job& job, std::string& message)
{
  std::vector<std::string> columns = { "name", "description", "part_id", "paths" };
  std::vector<std::string> values;
  values.push_back(boost::lexical_cast<std::string>(job.name));
  values.push_back(boost::lexical_cast<std::string>(job.description));
  values.push_back(boost::lexical_cast<std::string>(job.part_id));

  std::string filepath =
      save_dir_ + "/job_" + job.name + "_" + boost::posix_time::to_iso_string(ros::Time::now().toBoost()) + ".yaml";
  bool success = message_serialization::serialize(filepath, job.paths);
  if (!success)
  {
    message = "Failed to save tool paths to file at '" + filepath + "'";
    return -1;
  }

  values.push_back(boost::lexical_cast<std::string>(filepath));

  return insert(JOBS_TABLE_NAME, columns, values, message);
}

long int ROSDatabaseInterface::addPartToDatabase(const opp_msgs::Part& part, std::string& message)
{
  std::vector<std::string> columns = {
    "name", "description", "mesh_resource", "touchoff_points", "verification_points"
  };
  std::vector<std::string> values;
  values.push_back(boost::lexical_cast<std::string>(part.name));
  values.push_back(boost::lexical_cast<std::string>(part.description));
  values.push_back(boost::lexical_cast<std::string>(part.mesh_resource));

  const std::string time = boost::posix_time::to_iso_string(ros::Time::now().toBoost());

  // touchoff_points
  std::string filepath = save_dir_ + "/touch_pts_" + part.name + "_" + time + ".yaml";
  bool success = message_serialization::serialize(filepath, part.touch_points);
  if (!success)
  {
    message = "Failed to save touchoff points to file '" + filepath + "'";
    return -1;
  }

  values.push_back(boost::lexical_cast<std::string>(filepath));

  // verification_points
  filepath = save_dir_ + "/verification_points_" + part.name + "_" + time + ".yaml";
  success = message_serialization::serialize(filepath, part.verification_points);
  if (!success)
  {
    message = "Failed to save verification points to file '" + filepath + "'";
    return -1;
  }

  values.push_back(boost::lexical_cast<std::string>(filepath));

  return insert(PARTS_TABLE_NAME, columns, values, message);
}

bool ROSDatabaseInterface::getJobFromDatabase(const unsigned int job_id, std::string& message, opp_msgs::Job& job)
{
  // If the user tries to pull a specific suppressed job, let them
  QString script =
      QString("SELECT * FROM %1 WHERE `id`=\"%2\";").arg(QString::fromStdString(JOBS_TABLE_NAME)).arg(job_id);
  QSqlQuery result = database_.exec(script);

  if (!result.isActive())
  {
    message = "Database query failed: " + getErrorString(result);
    ;
    return false;
  }

  if (result.size() < 1)
  {
    message = "ID is not available";
    return false;
  }
  else if (result.size() > 1)
  {
    message = "Malformed table; multiple jobs share auto-generated ID";
    return false;
  }

  result.next();
  job.id = result.value("id").toUInt();
  job.name = result.value("name").toString().toStdString();
  job.description = result.value("description").toString().toStdString();
  job.part_id = result.value("part_id").toUInt();

  const std::string file_path = result.value("paths").toString().toStdString();
  bool success = message_serialization::deserialize(file_path, job.paths);
  if (!success)
  {
    message = "Failed to load tool paths file from '" + file_path + "'";
    return false;
  }

  message = "Successfully retrieved job from database";
  return success;
}

bool ROSDatabaseInterface::getAllJobsFromDatabase(const unsigned int part_id,
                                                  std::string& message,
                                                  std::map<unsigned int, opp_msgs::Job>& jobs)
{
  // Retrieve only non-suppressed jobs when jobs are requested en masse
  QString script = QString("SELECT * FROM %1 WHERE `part_id`=\"%2\" AND `suppressed`!=\"1\";")
                       .arg(QString::fromStdString(JOBS_TABLE_NAME))
                       .arg(part_id);
  QSqlQuery result = database_.exec(script);

  if (!result.isActive())
  {
    message = "Database query failed: " + getErrorString(result);
    ;
    return false;
  }

  if (result.size() < 1)
  {
    message = "No Jobs in the database for part id: " + std::to_string(part_id);
    return true;
  }

  jobs.clear();
  while (result.next())
  {
    opp_msgs::Job j;
    j.id = result.value("id").toUInt();
    j.name = result.value("name").toString().toStdString();
    j.description = result.value("description").toString().toStdString();
    j.part_id = result.value("part_id").toUInt();

    const std::string filepath = result.value("paths").toString().toStdString();
    bool success = message_serialization::deserialize(filepath, j.paths);
    if (!success)
    {
      message = "Failed to load tool path file from '" + filepath + "'";
      return false;
    }

    jobs.insert(std::pair<unsigned int, opp_msgs::Job>(j.id, j));
  }

  message = "Successfully retrieved full jobs for part id '" + std::to_string(part_id) + "' from database";
  return true;
}

bool ROSDatabaseInterface::getPartFromDatabase(const unsigned int part_id, std::string& message, opp_msgs::Part& part)
{
  // Let the user request a suppressed part specifically - they may need it for this job
  QString script =
      QString("SELECT * FROM %1 WHERE `part_id`=\"%2\";").arg(QString::fromStdString(PARTS_TABLE_NAME)).arg(part_id);
  QSqlQuery result = database_.exec(script);

  if (!result.isActive())
  {
    message = "Database query failed: " + getErrorString(result);
    ;
    return false;
  }

  if (result.size() < 1)
  {
    message = "ID is not available";
    return false;
  }
  else if (result.size() > 1)
  {
    message = "Malformed table; multiple parts share auto-generated ID";
    return false;
  }

  result.next();
  part.id = result.value("part_id").toUInt();
  part.name = result.value("name").toString().toStdString();
  part.description = result.value("description").toString().toStdString();
  part.mesh_resource = result.value("mesh_resource").toString().toStdString();

  const std::string touchoff_pts_filename = result.value("touchoff_points").toString().toStdString();
  bool success = message_serialization::deserialize(touchoff_pts_filename, part.touch_points);
  if (!success)
  {
    message = "Failed to load touchoff points from '" + touchoff_pts_filename + "'";
    return false;
  }

  const std::string verification_pts_filename = result.value("verification_points").toString().toStdString();
  success = message_serialization::deserialize(verification_pts_filename, part.verification_points);
  if (!success)
  {
    message = "Failed to load verification points from '" + verification_pts_filename + "'";
    return false;
  }

  message = "Successfully retrieved part from database";
  return success;
}

bool ROSDatabaseInterface::getAllPartsFromDatabase(std::string& message, std::map<unsigned int, opp_msgs::Part>& parts)
{
  // Do not retrieve suppressed parts when retrieved en masse
  QString script = QString("SELECT * FROM %1 WHERE `suppressed`!=\"1\";").arg(QString::fromStdString(PARTS_TABLE_NAME));
  QSqlQuery result = database_.exec(script);

  if (!result.isActive())
  {
    message = "Database query failed: " + getErrorString(result);
    ;
    return false;
  }

  if (result.size() < 1)
  {
    message = "No Parts in the database";
    return true;
  }

  parts.clear();
  while (result.next())
  {
    opp_msgs::Part p;
    p.id = result.value("part_id").toUInt();
    p.name = result.value("name").toString().toStdString();
    p.description = result.value("description").toString().toStdString();
    p.mesh_resource = result.value("mesh_resource").toString().toStdString();

    const std::string touch_pts_filename = result.value("touchoff_points").toString().toStdString();
    bool success = message_serialization::deserialize(touch_pts_filename, p.touch_points);
    if (!success)
    {
      message = "Failed to load touchoff points from '" + touch_pts_filename + "'";
      return false;
    }

    const std::string verification_pts_filename = result.value("verification_points").toString().toStdString();
    success = message_serialization::deserialize(verification_pts_filename, p.verification_points);
    if (!success)
    {
      message = "Failed to load verification points from '" + verification_pts_filename + "'";
      return false;
    }
    parts.insert(std::pair<unsigned int, opp_msgs::Part>(p.id, p));
  }

  message = "Successfully retrieved full parts from database";
  return true;
}

bool ROSDatabaseInterface::suppressPart(const unsigned int part_id, std::string& message)
{
  QString script = QString("UPDATE %1 SET `suppressed`=\"1\" WHERE `part_id`=\"%2\";")
                       .arg(QString::fromStdString(PARTS_TABLE_NAME))
                       .arg(part_id);
  QSqlQuery result = database_.exec(script);

  if (!result.isActive())
  {
    message = "Database query failed: " + getErrorString(result);
    ;
    return false;
  }

  result.next();

  message = "Successfully suppressed part in database";
  return true;
}

bool ROSDatabaseInterface::suppressJob(const unsigned int job_id, std::string& message)
{
  QString script = QString("UPDATE %1 SET `suppressed`=\"1\" WHERE `id`=\"%2\";")
                       .arg(QString::fromStdString(JOBS_TABLE_NAME))
                       .arg(job_id);
  QSqlQuery result = database_.exec(script);

  if (!result.isActive())
  {
    message = "Database query failed: " + getErrorString(result);
    ;
    return false;
  }

  result.next();

  message = "Successfully suppressed job in database";
  return true;
}

long ROSDatabaseInterface::insert(const std::string& table_name,
                                  const std::vector<std::string>& columns,
                                  const std::vector<std::string>& values,
                                  std::string& message)
{
  QString columns_string, values_string;
  for (const auto& column : columns)
    columns_string += QString("`%1`,").arg(QString::fromStdString(column));

  columns_string.chop(1);

  for (const auto& value : values)
    values_string += QString("\"%1\",").arg(QString::fromStdString(value));

  values_string.chop(1);

  QString script = QString("INSERT INTO %1 (%2) VALUES (%3);")
                       .arg(QString::fromStdString(table_name))
                       .arg(columns_string)
                       .arg(values_string);

  QSqlQuery result = database_.exec(script);
  if (result.isActive())
  {
    message = "Succesfully added item to table: " + table_name;
    return getLastEntryId(table_name);
  }
  else
  {
    message = "Failed to added item to table '" + table_name + "' " + getErrorString(result);
    return -1;
  }
}

bool ROSDatabaseInterface::createJobsTable()
{
  QString table_name = QString::fromStdString(JOBS_TABLE_NAME);
  QString script = QString("CREATE TABLE `%1` (\
      `id` int(10) unsigned NOT NULL AUTO_INCREMENT,\
      `name` varchar(100) NOT NULL,\
      `description` text NOT NULL,\
      `part_id` int(10) unsigned NOT NULL,\
      `paths` varchar(200) NOT NULL,\
      `suppressed` BOOLEAN NOT NULL DEFAULT 0,\
      PRIMARY KEY (`id`)\
    ) ENGINE=InnoDB AUTO_INCREMENT=9 DEFAULT CHARSET=latin1")
                       .arg(table_name);

  return createTableHelper(table_name, script);
}

bool ROSDatabaseInterface::createPartsTable()
{
  QString table_name = QString::fromStdString(PARTS_TABLE_NAME);
  QString script = QString("CREATE TABLE `%1` (\
      `part_id` int(10) unsigned NOT NULL AUTO_INCREMENT,\
      `name` varchar(100) NOT NULL,\
      `description` text NOT NULL,\
      `mesh_resource` varchar(200) DEFAULT NULL,\
      `touchoff_points` varchar(200) NOT NULL,\
      `verification_points` varchar(200) NOT NULL,\
      `suppressed` BOOLEAN NOT NULL DEFAULT 0,\
      PRIMARY KEY (`part_id`)\
    ) ENGINE=InnoDB AUTO_INCREMENT=27 DEFAULT CHARSET=latin1")
                       .arg(table_name);

  return createTableHelper(table_name, script);
}

bool ROSDatabaseInterface::createTableHelper(const QString& table_name, const QString& script)
{
  QStringList tables = database_.tables();

  // If the table does not exist it is created
  if (!tables.contains(table_name))
  {
    QSqlQuery result = database_.exec(script);

    if (result.isActive())
    {
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Failed to create table: '" + table_name.toStdString() + "' " + getErrorString(result));
      return false;
    }
  }

  return true;
}

std::string ROSDatabaseInterface::getErrorString(QSqlQuery& query) const
{
  return "DB Error (" + std::to_string(query.lastError().number()) + "): " + query.lastError().text().toStdString();
}

long int ROSDatabaseInterface::getLastEntryId(const std::string& table_name)
{
  QString id_field = "id";
  if (table_name == PARTS_TABLE_NAME)
    id_field = "part_id";

  QString script = QString("SELECT MAX(%1) FROM %2;").arg(id_field).arg(QString::fromStdString(table_name));

  QSqlQuery result = database_.exec(script);

  if (result.isActive() && result.size() > 0)
  {
    result.next();
    return result.value(0).toInt();
  }

  return -1;
}

}  // namespace opp_db
