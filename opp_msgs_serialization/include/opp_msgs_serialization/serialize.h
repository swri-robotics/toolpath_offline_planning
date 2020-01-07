/* 
 * Copyright 2018 Southwest Research Institute
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
#ifndef OPP_MSGS_SERIALIZATION_SERIALIZE_H
#define OPP_MSGS_SERIALIZATION_SERIALIZE_H

#include <fstream>

#include <yaml-cpp/yaml.h>

#include <ros/console.h>

namespace opp_msgs_serialization
{

template <class T>
bool serialize(const std::string &file, const T& val)
{
  std::ofstream ofh (file);
  if (!ofh)
  {
    return false;
  }

  YAML::Node n = YAML::Node(val);
  ofh << n;
  return true;
}

template <class T>
bool deserialize(const std::string &file, T& val)
{
  try
  {
    YAML::Node node;
    node = YAML::LoadFile(file);
    val = node.as<T>();
    return true;
  }
  catch (const YAML::Exception& ex)
  {
    ROS_ERROR_STREAM("An exception was thrown while processing file: " << file);
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
}

} // namespace opp_msgs_serialization

#endif // OPP_MSGS_SERIALIZATION_SERIALIZE_H
