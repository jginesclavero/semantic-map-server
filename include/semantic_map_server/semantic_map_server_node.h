/*
 * semantic_map_server_node.h
 *
 *  Created on: 27/07/2018
 *      Author: Jonathan Ginés
 */

#ifndef SEMANTICMAPSERVERNODE_H_
#define SEMANTICMAPSERVERNODE_H_

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
#include "nav_msgs/MapMetaData.h"


class SemanticMapServer {
public:
	SemanticMapServer();
	void step();

private:
	ros::NodeHandle nh_;
	costmap_2d::Costmap2D semantic_costmap_;
	costmap_2d::Costmap2DPublisher costmap_pub_;
	std::string map_file_;
	int height_,width_;
	float resolution_,origin_x_,origin_y_;

	void loadMap(int height, int width);
	void initSemanticMap();
	void removeHeaders(FILE* &file);
};

#endif /* SEMANTICMAPSERVERNODE_H_ */
