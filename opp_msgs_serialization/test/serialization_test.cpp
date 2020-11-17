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

#include <opp_msgs_serialization/opp_msgs_yaml.h>
#include <message_serialization/serialize.h>
#include <gtest/gtest.h>

TEST(opp_serialization, process_type)
{
  std::string file = "process_type.yaml";
  opp_msgs::ProcessType orig_msg;
  orig_msg.val = opp_msgs::ProcessType::PROCESS_PAINT;

  bool result = message_serialization::serialize(file, orig_msg);
  EXPECT_TRUE(result);

  opp_msgs::ProcessType new_msg;
  bool load_result = message_serialization::deserialize(file, new_msg);
  EXPECT_TRUE(load_result);

  bool eq = (orig_msg.val == new_msg.val);
  EXPECT_TRUE(eq);
}

TEST(opp_serialization, touch_point)
{
  std::string file = "touch_point.yaml";
  opp_msgs::TouchPoint orig_msg;
  orig_msg.name = "test";
  orig_msg.description = "test";
  orig_msg.transform.pose.position.x = 1.0;
  orig_msg.transform.pose.position.y = 1.0;
  orig_msg.transform.pose.position.z = 1.0;
  orig_msg.transform.pose.orientation.w = 1.0;
  orig_msg.transform.pose.orientation.x = 1.0;
  orig_msg.transform.pose.orientation.y = 1.0;
  orig_msg.transform.pose.orientation.z = 1.0;

  bool save_result = message_serialization::serialize(file, orig_msg);
  EXPECT_TRUE(save_result);

  opp_msgs::TouchPoint new_msg;
  bool load_result = message_serialization::deserialize(file, new_msg);
  EXPECT_TRUE(load_result);

  EXPECT_EQ(new_msg.name, orig_msg.name);
  EXPECT_EQ(new_msg.description, orig_msg.description);
  EXPECT_EQ(new_msg.transform.pose.position.x, orig_msg.transform.pose.position.x);
  EXPECT_EQ(new_msg.transform.pose.position.y, orig_msg.transform.pose.position.y);
  EXPECT_EQ(new_msg.transform.pose.position.z, orig_msg.transform.pose.position.z);
  EXPECT_EQ(new_msg.transform.pose.orientation.w, orig_msg.transform.pose.orientation.w);
  EXPECT_EQ(new_msg.transform.pose.orientation.x, orig_msg.transform.pose.orientation.x);
  EXPECT_EQ(new_msg.transform.pose.orientation.y, orig_msg.transform.pose.orientation.y);
  EXPECT_EQ(new_msg.transform.pose.orientation.z, orig_msg.transform.pose.orientation.z);
}

TEST(opp_serialization, tool_path)
{
  std::string file = "tool_path.yaml";
  opp_msgs::ToolPath orig_msg;

  orig_msg.process_type.val = opp_msgs::ProcessType::PROCESS_DEPAINT;

  for (unsigned segment_idx = 0; segment_idx < 3; ++segment_idx)
  {
    geometry_msgs::PoseArray segment;

    for (unsigned point_idx = 0; point_idx < 10; ++point_idx)
    {
      geometry_msgs::Pose pose;
      pose.position.x = static_cast<double>(point_idx);
      pose.position.y = static_cast<double>(point_idx);
      ;
      pose.position.z = static_cast<double>(point_idx);
      ;
      pose.orientation.w = static_cast<double>(point_idx);
      ;
      pose.orientation.x = static_cast<double>(point_idx);
      ;
      pose.orientation.y = static_cast<double>(point_idx);
      ;
      pose.orientation.z = static_cast<double>(point_idx);
      ;

      segment.poses.push_back(std::move(pose));
    }

    orig_msg.paths.push_back(std::move(segment));
  }

  bool result = message_serialization::serialize(file, orig_msg);
  EXPECT_TRUE(result);

  opp_msgs::ToolPath new_msg;
  bool load_result = message_serialization::deserialize(file, new_msg);
  EXPECT_TRUE(load_result);

  EXPECT_EQ(new_msg.process_type.val, orig_msg.process_type.val);
  EXPECT_EQ(new_msg.paths.size(), orig_msg.paths.size());

  EXPECT_EQ(new_msg.tool_offset.position.x, orig_msg.tool_offset.position.x);
  EXPECT_EQ(new_msg.tool_offset.position.y, orig_msg.tool_offset.position.y);
  EXPECT_EQ(new_msg.tool_offset.position.z, orig_msg.tool_offset.position.z);
  EXPECT_EQ(new_msg.tool_offset.orientation.w, orig_msg.tool_offset.orientation.w);
  EXPECT_EQ(new_msg.tool_offset.orientation.x, orig_msg.tool_offset.orientation.x);
  EXPECT_EQ(new_msg.tool_offset.orientation.y, orig_msg.tool_offset.orientation.y);
  EXPECT_EQ(new_msg.tool_offset.orientation.z, orig_msg.tool_offset.orientation.z);

  for (unsigned segment_idx = 0; segment_idx < new_msg.paths.size(); ++segment_idx)
  {
    const geometry_msgs::PoseArray& new_segment = new_msg.paths[segment_idx];
    const geometry_msgs::PoseArray& orig_segment = orig_msg.paths[segment_idx];
    EXPECT_EQ(new_segment.poses.size(), orig_segment.poses.size());

    for (unsigned point_idx = 0; point_idx < new_segment.poses.size(); ++point_idx)
    {
      const geometry_msgs::Pose& new_pose = new_msg.paths[segment_idx].poses[point_idx];
      const geometry_msgs::Pose& orig_pose = orig_msg.paths[segment_idx].poses[point_idx];

      EXPECT_EQ(new_pose.position.x, orig_pose.position.x);
      EXPECT_EQ(new_pose.position.y, orig_pose.position.y);
      EXPECT_EQ(new_pose.position.z, orig_pose.position.z);
      EXPECT_EQ(new_pose.orientation.w, orig_pose.orientation.w);
      EXPECT_EQ(new_pose.orientation.x, orig_pose.orientation.x);
      EXPECT_EQ(new_pose.orientation.y, orig_pose.orientation.y);
      EXPECT_EQ(new_pose.orientation.z, orig_pose.orientation.z);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
