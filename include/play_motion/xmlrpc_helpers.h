/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Paul Mathieu. */

#ifndef XMLRPCHELPERS_H
#define XMLRPCHELPERS_H

#include <string>
#include <ros/ros.h>

#include "play_motion/rethrow.h"

namespace xh
{
  bool checkParamType(const XmlRpc::XmlRpcValue& val, const XmlRpc::XmlRpcValue::Type& expected_type,
      const std::string& param_name = "")
  {
    if (val.getType() != expected_type)
    {
      if (param_name.empty())
        ROS_ERROR("wrong parameter type");
      else
        ROS_ERROR("wrong parameter type for param '%s'", param_name.c_str());
      return false;
    }
    return true;
  }

  template <class T>
    bool fetchParam(ros::NodeHandle nh, const std::string& param_name,
        const XmlRpc::XmlRpcValue::Type& expected_type, T& output)
    {
      XmlRpc::XmlRpcValue val;
      if (!nh.getParamCached(param_name, val))
      {
        ROS_ERROR("could not load parameter '%s'. (namespace: %s)",
            param_name.c_str(), nh.getNamespace().c_str());
        return false;
      }
      RETHROW(checkParamType(val, expected_type, param_name));
      output = static_cast<T>(val);
      return true;
    }

  bool checkSubItem(const XmlRpc::XmlRpcValue& col, int index)
  {
    RETHROW(checkParamType(col, XmlRpc::XmlRpcValue::TypeArray));
    if(index < col.size())
      return true;
    ROS_ERROR("index '%d' is over array capacity", index);
    return false;
  }
  bool checkSubItem(const XmlRpc::XmlRpcValue& col, const std::string& member)
  {
    RETHROW(checkParamType(col, XmlRpc::XmlRpcValue::TypeStruct));
    if (col.hasMember(member))
      return true;
    ROS_ERROR("could not find member '%s'", member.c_str());
    return false;
  }

  template <class T, class U>
    bool getSubItem(XmlRpc::XmlRpcValue& col, U member,
        const XmlRpc::XmlRpcValue::Type& expected_type, T& output)
    {
      RETHROW(checkSubItem(col, member));
      RETHROW(checkParamType(col[member], expected_type));
      output = static_cast<T>(col[member]);
      return true;
    }

#define getArrayItem getSubItem
#define getStructMember getSubItem
}

#endif
