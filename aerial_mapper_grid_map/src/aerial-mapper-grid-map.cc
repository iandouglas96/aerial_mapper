/*
 *    Filename: aerial-mapper-grid-map.cc
 *  Created on: Oct 9, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "aerial-mapper-grid-map/aerial-mapper-grid-map.h"

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace grid_map {

AerialGridMap::AerialGridMap(const Settings& settings)
    : settings_(settings),
      node_handle_{},
      pub_grid_map_(
          node_handle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true)),
      pub_map_image_(
          node_handle_.advertise<sensor_msgs::Image>("map_image", 1, true)),
      pub_map_image_viz_(
          node_handle_.advertise<sensor_msgs::Image>("map_image_viz", 1, true)),
      pub_map_loc_(
          node_handle_.advertise<geometry_msgs::PointStamped>("map_loc", 1, true)) {
  initialize();
}

void AerialGridMap::initialize() {
  // Create grid map.
  map_ = grid_map::GridMap({"ortho", "elevation", "elevation_angle",
                            "num_observations", "elevation_angle_first_view",
                            "delta", "observation_index",
                            "observation_index_first","colored_ortho"});
  map_.setFrameId("world");
  map_.setGeometry(
      grid_map::Length(settings_.delta_easting, settings_.delta_northing),
      settings_.resolution,
      grid_map::Position(settings_.center_easting, settings_.center_northing));
  ROS_INFO(
      "Created map with size %f x %f m (%i x %i cells).\n The center of the "
      "map is located at (%f, %f) in the %s frame.",
      map_.getLength().x(), map_.getLength().y(), map_.getSize()(0),
      map_.getSize()(1), map_.getPosition().x(), map_.getPosition().y(),
      map_.getFrameId().c_str());

  init_pos_ = getCornerPos();

  //LUT for coloring for viz
  color_lut_ = cv::Mat(256, 1, CV_8UC3, cv::Scalar(255,255,255));
  color_lut_.at<cv::Vec3b>(102) = cv::Vec3b(0,100,0);     //terrain
  color_lut_.at<cv::Vec3b>(100) = cv::Vec3b(255,0,0);     //road
  color_lut_.at<cv::Vec3b>(101) = cv::Vec3b(255,0,255);   //dirt
  color_lut_.at<cv::Vec3b>(2) = cv::Vec3b(0,0,255);       //building
  color_lut_.at<cv::Vec3b>(7) = cv::Vec3b(0,255,0);       //veg
  color_lut_.at<cv::Vec3b>(13) = cv::Vec3b(255,255,0);    //car

  map_.setConstant("ortho", 0);
  map_.setConstant("elevation", NAN);
  map_.setConstant("elevation_angle", 0.0);
  map_.setConstant("elevation_angle_first_view", NAN);
  map_.setConstant("num_observations", 0);
  map_.setConstant("observation_index", NAN);
  map_.setConstant("observation_index_first", NAN);
  map_.setConstant("delta", NAN);
  map_.setConstant("colored_ortho", NAN);
  /*
  map_["ortho"].setConstant(255);
  map_["elevation"].setConstant(NAN);
  map_["elevation_angle"].setConstant(0.0);
  map_["elevation_angle_first_view"].setConstant(0);
  map_["num_observations"].setConstant(0);
  map_["observation_index"].setConstant(0);
  map_["observation_index_first"].setConstant(0);
  map_["delta"].setConstant(0);
  map_["colored_ortho"].setConstant(0);
  */
}

Position AerialGridMap::getCornerPos() {
  Position pos;
  pos[0] = map_.getPosition()[1] + map_.getLength()[1]/2;
  pos[1] = map_.getPosition()[0] - map_.getLength()[0]/2;
  return pos;
}

void AerialGridMap::publishUntilShutdown() {
  ros::Rate r(0.1);  // 10 hz
  ros::NodeHandle node_handle;
  ros::Publisher pub_grid_map =
      node_handle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  while (true) {
    map_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    pub_grid_map.publish(message);
    ros::spinOnce();
    r.sleep();
  }
}

void AerialGridMap::publishOnce() {
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  pub_grid_map_.publish(message);
  //ros::spinOnce();
}

void AerialGridMap::publishImageOnce() {
  cv::Mat img(map_.getSize()(0), map_.getSize()(1), CV_8UC3, cv::Scalar(255,255,255));
  cv::Mat img_viz(map_.getSize()(0), map_.getSize()(1), CV_8UC3, cv::Scalar(255,255,255));

  //The default toImage does not properly handle color
  const grid_map::Matrix& data = map_["colored_ortho"];
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    const float& value = data(index(0), index(1));
    if (std::isfinite(value)) {
      const grid_map::Index image_index(it.getUnwrappedIndex());
      Eigen::Vector3f color_vec;
      grid_map::colorValueToVector(value, color_vec);
      img.at<cv::Vec<uint8_t, 3>>(image_index(0), image_index(1)) = 
        cv::Vec<uint8_t, 3>(color_vec[0]*255, color_vec[1]*255, color_vec[2]*255);
    }
  }

  cv_bridge::CvImagePtr img_ptr(new cv_bridge::CvImage);
  img_ptr->image = img;
  img_ptr->encoding = "rgb8";
  sensor_msgs::Image::Ptr img_msg = img_ptr->toImageMsg();
  pub_map_image_.publish(img_msg); 

  cv::LUT(img, color_lut_, img_viz);
  img_ptr->image = img_viz;
  pub_map_image_viz_.publish(img_ptr->toImageMsg()); 

  //position of the center of the map in meters
  Position pos = (getCornerPos().array() - init_pos_.array())/map_.getResolution();
    
  geometry_msgs::PointStamped map_origin;
  map_origin.header = img_msg->header;
  map_origin.point.x = pos.x();
  map_origin.point.y = -pos.y();
  pub_map_loc_.publish(map_origin);
}

}  // namespace grid_map
