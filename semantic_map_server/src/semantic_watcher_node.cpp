/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2021, Intelligent Robotics
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

#include <semantic_map_server/semantic_watcher_node.h>

SemanticWatcher::SemanticWatcher(): nh_("~"), costmap_() {
	map_sub_ = nh_.subscribe("/semantic_map", 1, &SemanticWatcher::mapCb, this);
	metadata_sub_ = nh_.subscribe("/semantic_map_metadata",
		1, &SemanticWatcher::metadataCallback, this);
  robot_location_pub_ = nh_.advertise<std_msgs::String>("/semantic_robot_location", 1);
	map_ready_ = false;
	metadata_ready_ = false;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void SemanticWatcher::mapCb(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	grid2CostMap(*map,costmap_);
	map_ready_ = true;
}

void SemanticWatcher::metadataCallback(
	const semantic_map_server_msgs::SemanticMapMetaData::ConstPtr& metadata) 
{
	rooms_ = metadata->rooms;
	for (int i = 0; i < rooms_.size(); i++)
		semantic_values_[metadata->semantic_values[i]] = rooms_[i];
	metadata_ready_ = true;
}

void 
SemanticWatcher::grid2CostMap(nav_msgs::OccupancyGrid map,
	costmap_2d::Costmap2D& cost_map) 
{
	int i = 0;
	float normalizer = 2.54;
	cost_map.resizeMap(map.info.width,map.info.height, map.info.resolution,
		map.info.origin.position.x, map.info.origin.position.y);
	cost_map.setDefaultValue(254);
	for (int y = 0; y < cost_map.getSizeInCellsY(); y++) {
		for (int x = 0; x < cost_map.getSizeInCellsX(); x++) {
			if (map.data[i] == -1)
				cost_map.setCost(x, y, 255);
			else
				cost_map.setCost(x, y, round(normalizer*map.data[i]));
			i++;
		}
	}
}

std::string 
SemanticWatcher::getRobotLocation(geometry_msgs::TransformStamped map2robot_msg)
{
  unsigned int cell_x, cell_y;
	unsigned char cell_cost;
	costmap_.worldToMap(
    map2robot_msg.transform.translation.x,
	  map2robot_msg.transform.translation.y, 
    cell_x, 
    cell_y);
	cell_cost = costmap_.getCost(cell_x, cell_y);
	return semantic_values_[round(cell_cost / 2.54)];
}


bool 
SemanticWatcher::getRobotTF(geometry_msgs::TransformStamped& map2robot_msg)
{
	try 
  {
    map2robot_msg = tf_buffer_->lookupTransform("map", "base_footprint" , ros::Time(0));
	} catch (tf2::TransformException & ex) {
		ROS_ERROR("[semantic_watcher] %s", ex.what());
		return false;
	}
	return true;
}

void 
SemanticWatcher::step() 
{
  if (map_ready_ && metadata_ready_) 
  {
    geometry_msgs::TransformStamped map2robot;
    getRobotTF(map2robot);
    std_msgs::String msg;
    msg.data = getRobotLocation(map2robot);
    robot_location_pub_.publish(msg);
  }
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "semantic_watcher_node");
  ros::NodeHandle n;
  SemanticWatcher semantic_watcher;
	ros::Rate loop_rate(2);
 	while (ros::ok()){
		semantic_watcher.step();
 		ros::spinOnce();
 		loop_rate.sleep();
 	}
	return 0;
}

