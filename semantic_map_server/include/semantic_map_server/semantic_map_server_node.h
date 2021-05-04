/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonathan Ginés jgines@gsyc.urjc.es */

#ifndef SEMANTIC_MAP_SERVER_SEMANTIC_MAP_SERVER_NODE_H
#define SEMANTIC_MAP_SERVER_SEMANTIC_MAP_SERVER_NODE_H

#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <math.h>
#include <iostream>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <semantic_map_server_msgs/SemanticMapMetaData.h>

class SemanticMapServer
{
public:
  SemanticMapServer();

private:
  ros::NodeHandle nh_;
  costmap_2d::Costmap2D semantic_costmap_;
  costmap_2d::Costmap2DPublisher costmap_pub_;
  ros::Publisher metadata_pub_;
  std::string map_file_;
  int height_, width_;
  float resolution_, origin_x_, origin_y_;
  std::vector<std::string> rooms_;
  std::vector<int> values_;
  semantic_map_server_msgs::SemanticMapMetaData metadata_msg_;

  void loadParameters();
  void loadMap(int height, int width);
  void initSemanticMap();
  void removeHeaders(FILE* &file);
};

#endif /* SEMANTIC_MAP_SERVER_SEMANTIC_MAP_SERVER_NODE_H */
