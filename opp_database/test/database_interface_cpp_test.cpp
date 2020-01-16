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

#include <gtest/gtest.h>
#include <opp_database/opp_database_interface_cpp.h>

TEST(ROSDatabaseInterfaceCppUnit, GetTableLastID)
{
  opp_db::ROSDatabaseInterface database;
  //  long int id = database.getLastEntryId(opp_db::LOC_LOG_TABLE_NAME);
  //  EXPECT_TRUE(id == 7);
}

TEST(ROSDatabaseInterfaceCppUnit, GetAllJobs)
{
  opp_db::ROSDatabaseInterface database;
  const unsigned int part_id = 9999;
  std::string message;
  std::map<unsigned int, opp_msgs::Job> jobs;

  EXPECT_TRUE(database.getAllJobsFromDatabase(part_id, message, jobs));
  EXPECT_TRUE(jobs.size() == 2);
}

TEST(ROSDatabaseInterfaceCppUnit, GetAllParts)
{
  opp_db::ROSDatabaseInterface database;
  std::string message;
  std::map<unsigned int, opp_msgs::Part> parts;

  EXPECT_TRUE(database.getAllPartsFromDatabase(message, parts));
  EXPECT_TRUE(parts.size() > 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
