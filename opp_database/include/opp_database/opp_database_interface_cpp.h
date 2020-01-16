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

#ifndef OPP_DATABASE_INTERAFACE_CPP_H
#define OPP_DATABASE_INTERAFACE_CPP_H

#include <vector>
#include <string>

#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>

#include <opp_msgs/Job.h>
#include <opp_msgs/Part.h>

namespace opp_db
{
const static std::string JOBS_TABLE_NAME = "jobs";
const static std::string PARTS_TABLE_NAME = "parts";

class ROSDatabaseInterface
{
public:
  ROSDatabaseInterface();
  virtual ~ROSDatabaseInterface();

  QSqlDatabase& getDatabase();
  bool isConnected() const;

  long int addJobToDatabase(const opp_msgs::Job& job, std::string& message);

  long int addPartToDatabase(const opp_msgs::Part& part, std::string& message);

  bool getJobFromDatabase(const unsigned int part_id, std::string& message, opp_msgs::Job& job);

  bool getAllJobsFromDatabase(const unsigned int part_id,
                              std::string& message,
                              std::map<unsigned int, opp_msgs::Job>& jobs);

  bool getPartFromDatabase(const unsigned int part_id, std::string& message, opp_msgs::Part& part);

  bool getAllPartsFromDatabase(std::string& message, std::map<unsigned int, opp_msgs::Part>& parts);

  bool suppressJob(const unsigned int job_id, std::string& message);

  bool suppressPart(const unsigned int part_id, std::string& message);

  long int getLastEntryId(const std::string& table_name);

  std::string getErrorString(QSqlQuery& query) const;

protected:
  QSqlDatabase database_;
  std::string save_dir_;

  long int insert(const std::string& table_name,
                  const std::vector<std::string>& columns,
                  const std::vector<std::string>& values,
                  std::string& message);

  bool createJobsTable();

  bool createPartsTable();

  bool createTableHelper(const QString& table_name, const QString& script);
};

}  // namespace opp_db
#endif  // OPP_DATABASE_INTERAFACE_CPP_H
