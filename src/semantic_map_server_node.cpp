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

#include <semantic_map_server/semantic_map_server_node.h>

SemanticMapServer::SemanticMapServer()
  : nh_("~"),
    semantic_costmap_(),
    costmap_pub_(&nh_, &semantic_costmap_, "/map", "/semantic_map", true),
    map_file_(),
    metadata_msg_(),
    values_()
    {
  metadata_pub_ = nh_.advertise<semantic_map_server::SemanticMapMetaData>(
    "/semantic_map_metadata", 1, true);
  loadParameters();
  loadMap(height_, width_);
  costmap_pub_.publishCostmap();
  metadata_pub_.publish(metadata_msg_);
}

void SemanticMapServer::loadParameters()
{
  if (!nh_.getParam("image", map_file_))
    ROS_ERROR("Do not find image param");
  nh_.getParam("height", height_);
  nh_.getParam("width", width_);
  nh_.getParam("resolution", resolution_);
  nh_.getParam("origin_x", origin_x_);
  nh_.getParam("origin_y", origin_y_);
  if (nh_.hasParam("locations/room_ids"))
  {
    nh_.getParam("locations/room_ids", rooms_);
    for (int i = 0; i < rooms_.size(); i++)
    {
      if (nh_.hasParam("locations/"+rooms_[i]))
      {
        int value;
        nh_.getParam("locations/"+rooms_[i], value);
        values_.push_back(value);
      }
    }
  }
  metadata_msg_.rooms = rooms_;
  metadata_msg_.semantic_values = values_;
}

void SemanticMapServer::loadMap(int height, int width)
{
  const int kMaxLineLen = 100;
  char line[kMaxLineLen];
  uint cell_cost;
  initSemanticMap();
  FILE* file = fopen(map_file_.c_str(), "r");
  if (!file)
  {
    ROS_ERROR("Couldn't load semanftic map file to %s", map_file_.c_str());
    return;
  }
  removeHeaders(file);
  for (int i = height-1; i >= 0; i--)
  {
    for (int j = 0; j < width; j++)
    {
      if (fgets(line, kMaxLineLen, file))
      {
        cell_cost = atoi(line) + 1;
        if (cell_cost < 0 || cell_cost == 255)
          cell_cost = 254;
        semantic_costmap_.setCost(j, i, cell_cost);
      }
    }
  }
  fclose(file);
}

void SemanticMapServer::initSemanticMap()
{
  semantic_costmap_.resizeMap(
    width_,
    height_,
    resolution_,
    origin_x_,
    origin_y_);
  semantic_costmap_.setDefaultValue(0);
  semantic_costmap_.resetMap(
    0,
    0,
    semantic_costmap_.getSizeInCellsX(),
    semantic_costmap_.getSizeInCellsY());
}

void SemanticMapServer::removeHeaders(FILE* &file)
{
  int lines_to_rm = 3;
  const int kMaxLineLen = 100;
  char line[kMaxLineLen];
  for (int i = 0; i < lines_to_rm; i++)
  {
    fgets(line, kMaxLineLen, file);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_map_server_node");
  SemanticMapServer semantic_map_server;
  ros::spin();
  return 0;
}
