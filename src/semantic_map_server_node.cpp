/*
 *
 * Author: Jonathan Ginés Clavero (jonathangines@hotmail.com)
 * Date: 27/07/2018
 *
 * Description: Node to load a semantic map from disk and offer it in a topic.
 *
 */

#include <semantic_map_server/semantic_map_server_node.h>

SemanticMapServer::SemanticMapServer()
  : nh_("~"),
    semantic_costmap_(),
    costmap_pub_(&nh_, &semantic_costmap_, "/map", "/semantic_map", true),
    map_file_() {
  if (!nh_.getParam("map_file", map_file_))
    ROS_ERROR("Do not find map_file param");
  nh_.getParam("height", height_);
  nh_.getParam("width", width_);
  nh_.getParam("resolution", resolution_);
  nh_.getParam("origin_x", origin_x_);
  nh_.getParam("origin_y", origin_y_);
  loadMap(height_,width_);
}

void SemanticMapServer::loadMap(int height, int width) {
  const int kMaxLineLen = 100;
  char line[kMaxLineLen];
  uint cell_cost;

  initSemanticMap();

  FILE* file = fopen(map_file_.c_str(),"r");
  if (!file) {
      ROS_ERROR("Couldn't load semantic map file to %s", map_file_.c_str());
      return;
  }
  removeHeaders(file);

  for (int i = height-1; i >= 0; i--) {
    for (int j = 0; j<width; j++) {
      if (fgets(line, kMaxLineLen, file)) {
        cell_cost = 254 - atoi(line);
        if (cell_cost < 0 || cell_cost == 255)
          cell_cost = 254;
        semantic_costmap_.setCost(j, i, cell_cost);
      }
    }
  }
  fclose(file);
}

void SemanticMapServer::initSemanticMap() {
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

void SemanticMapServer::removeHeaders(FILE* &file){
  int lines_to_rm = 3;
  const int kMaxLineLen = 100;
  char line[kMaxLineLen];
  for (int i = 0; i < lines_to_rm; i++){
    fgets(line, kMaxLineLen, file);
  }
}

void SemanticMapServer::step() {
  costmap_pub_.publishCostmap();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "semantic_map_server_node");
  SemanticMapServer semantic_map_server;
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    semantic_map_server.step();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
