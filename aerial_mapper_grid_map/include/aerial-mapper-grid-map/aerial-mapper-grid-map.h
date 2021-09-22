/*
 *    Filename: aerial-mapper-grid-map.h
 *  Created on: Oct 9, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef AERIAL_MAPPER_GRID_MAP_H_
#define AERIAL_MAPPER_GRID_MAP_H_


#include <Eigen/Dense>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/publisher.h>
#include <ros/ros.h>

namespace grid_map {

struct Settings {
  double center_easting;
  double center_northing;
  double delta_easting;
  double delta_northing;
  double resolution;
};

class AerialGridMap{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AerialGridMap(const Settings& settings);

  void publishUntilShutdown();

  void publishOnce();
  void publishImageOnce();

  grid_map::GridMap* getMutable() {
    return &map_;
  }

private:

  void initialize();
  Position getCornerPos();

  grid_map::GridMap map_;
  cv::Mat color_lut_;
  Position init_pos_;

  Settings settings_;
  ros::NodeHandle node_handle_;
  ros::Publisher pub_grid_map_, pub_map_image_, pub_map_image_viz_, pub_map_loc_;
};


} // namespace grid_map

#endif  // AERIAL_MAPPER_GRID_MAP_H_
