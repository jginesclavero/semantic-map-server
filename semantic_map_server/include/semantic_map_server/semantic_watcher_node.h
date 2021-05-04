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

/* Author: Jonathan Ginés jonatan.gines@urjc.es */

#ifndef SEMANTIC_MAP_SERVER_SEMANTIC_WATCHER_NODE_H
#define SEMANTIC_MAP_SERVER_SEMANTIC_WATCHER_NODE_H

#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <iostream>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include <semantic_map_server_msgs/SemanticMapMetaData.h>

class SemanticWatcher {
	public:
	SemanticWatcher();
	void step();
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& map);
	void metadataCallback(
	const semantic_map_server_msgs::SemanticMapMetaData::ConstPtr& map);

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_,metadata_sub_;
  ros::Publisher robot_location_pub_;
  costmap_2d::Costmap2D costmap_;
  std::vector<std::string> rooms_;
  std::map<int, std::string> semantic_values_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool map_ready_, metadata_ready_;

  bool getRobotTF(geometry_msgs::TransformStamped& map2robot_msg);
  std::string getRobotLocation(geometry_msgs::TransformStamped map2robot_msg);
	void grid2CostMap(nav_msgs::OccupancyGrid map, costmap_2d::Costmap2D& cost_map);
};

#endif /* SEMANTICWATCHERNODE_H_ */
