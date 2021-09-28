/*
 *    Filename: main-ortho-backward-grid-incremental.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// SYSTEM
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-dense-pcl/stereo.h>
#include <aerial-mapper-dsm/dsm.h>
#include <aerial-mapper-grid-map/aerial-mapper-grid-map.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-ortho/ortho-backward-grid.h>
#include <gflags/gflags.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

DEFINE_string(backward_grid_data_directory, "",
              "Directory to poses, images, and calibration file.");
DEFINE_string(backward_grid_filename_poses, "",
              "Name of the file that contains positions and orientations for "
              "every camera in the global/world frame, i.e. T_G_B");
DEFINE_string(backward_grid_prefix_images, "",
              "Prefix of the images to be loaded, e.g. 'images_'");
DEFINE_string(
    backward_grid_filename_camera_rig, "",
    "Name of the camera calibration file (intrinsics). File ending: .yaml");
DEFINE_double(backward_grid_center_easting, 0.0,
              "Center [m] of the grid_map (easting).");
DEFINE_double(backward_grid_center_northing, 0.0,
              "Center [m] of the grid_map (northing).");
DEFINE_double(backward_grid_delta_easting, 100.0,
              "Width [m] of the grid_map, starting from center.");
DEFINE_double(backward_grid_delta_northing, 100.0,
              "Height [m] of the grid_map, starting from center");
DEFINE_double(backward_grid_resolution, 1.0, "Resolution of the grid_map [m].");
DEFINE_bool(backward_grid_show_orthomosaic_opencv, true,
            "Show the orthomosaic using opencv?");
DEFINE_bool(backward_grid_save_orthomosaic_jpg, true,
            "Save the orthomosaic as jpg to file?");
DEFINE_string(backward_grid_orthomosaic_jpg_filename, "",
              "Name of the output image storing the orthomsaic.");
DEFINE_double(backward_grid_orthomosaic_elevation_m, 0.0,
              "Height of the orthomosaic if flat ground assumption is used.");
DEFINE_bool(backward_grid_use_digital_elevation_map, true,
            "Use the digital elevation map for generating the orthomosaic? "
            "Otherwise use flat ground assumption.");
DEFINE_string(point_cloud_filename, "",
              "Name of the file that contains the point cloud. If string is "
              "empty, the point cloud is generated from the provided images, "
              "camera poses, camera intrinsics");
DEFINE_int32(dense_pcl_use_every_nth_image, 1,
             "Only use every n-th image in the densification process.");
DEFINE_bool(backward_grid_colored_ortho, false,
            "Generate a colored (RGB) orthomosaic? Otherwise: grayscale.");
DEFINE_bool(backward_grid_use_multi_threads, false,
            "Use multi threads for orthomosaic generation?");
DEFINE_bool(use_BM, true,
            "Use BM Blockmatching if true. Use SGBM (=Semi-Global-) "
            "Blockmatching if false.");
DEFINE_bool(integrate_semantics, false,
            "Integrate semantics instead of RGB, use RGB for stereo.");
DEFINE_double(map_border, 10.0,
              "Distance of robot to border before adding to the map [m]");
DEFINE_double(stereo_dist_thresh, 1.0,
              "Distance of robot to move before attemping stereo depth [m]");

Images images_subset;
Poses T_G_Bs_subset;
size_t pcl_cnt = 0;
kindr::minimal::Position last_pos;

ortho::OrthoBackwardGrid *mosaic;
dsm::Dsm *digital_surface_map;
grid_map::AerialGridMap *map;
stereo::Stereo *stereo_proc;

void procResize(const kindr::minimal::Position& camera_pos) {
  Eigen::Vector2d map_center = map->getMutable()->getPosition();
  Eigen::Vector2d map_length = map->getMutable()->getLength();
  //check if camera is close to map borders
  ROS_WARN_STREAM(camera_pos);
  if (camera_pos.x() > map_center.x() + map_length.x()/2 - FLAGS_map_border) {
    map->getMutable()->grow(map_length + Eigen::Vector2d(50, 0), grid_map::GridMap::NW);
  } else if (camera_pos.x() < map_center.x() - map_length.x()/2 + FLAGS_map_border) {
    map->getMutable()->grow(map_length + Eigen::Vector2d(50, 0), grid_map::GridMap::SE);
  } else if (camera_pos.y() > map_center.y() + map_length.y()/2 - FLAGS_map_border) {
    map->getMutable()->grow(map_length + Eigen::Vector2d(0, 50), grid_map::GridMap::NW);
  } else if (camera_pos.y() < map_center.y() - map_length.y()/2 + FLAGS_map_border) {
    map->getMutable()->grow(map_length + Eigen::Vector2d(0, 50), grid_map::GridMap::SE);
  }
}

void imgCb(const sensor_msgs::Image::ConstPtr& img, 
           const sensor_msgs::Image::ConstPtr& sem_img,
           const geometry_msgs::PoseStamped::ConstPtr& pose) {
  LOG(INFO) << "Cb start";
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
  //convert to greyscale (or not)
  Image cv_img, cv_sem_img;
  if (!FLAGS_backward_grid_colored_ortho) {
    cv::cvtColor(cv_ptr->image, cv_img, cv::COLOR_BGR2GRAY);
  } else {
    cv_img = cv_ptr->image;
  }

  if (sem_img && FLAGS_backward_grid_colored_ortho) {
    cv_ptr = cv_bridge::toCvShare(sem_img, sensor_msgs::image_encodings::BGR8);
    cv_sem_img = cv_ptr->image;
  }
  
  kindr::minimal::Position pos(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
  kindr::minimal::RotationQuaternion rot(pose->pose.orientation.w,
                                         pose->pose.orientation.x,
                                         pose->pose.orientation.y,
                                         pose->pose.orientation.z);
  Pose T_G_B(pos, rot);

  //check if baseline is long enough
  if ((pos.head<2>() - last_pos.head<2>()).norm() > FLAGS_stereo_dist_thresh) {
    procResize(pos);

    if (sem_img && FLAGS_backward_grid_colored_ortho) {
      images_subset.push_back(cv_sem_img);
    } else {
      images_subset.push_back(cv_img);
    }
    T_G_Bs_subset.push_back(T_G_B);

    LOG(INFO) << "Processing image";
    AlignedType<std::vector, Eigen::Vector3d>::type point_cloud;
    stereo_proc->addFrame(T_G_B, cv_img, &point_cloud);
    LOG(INFO) << "Image proc complete";

    if (pcl_cnt > 0) {
      LOG(INFO) << "Filling DSM with " << point_cloud.size() << " points";
      digital_surface_map->process(point_cloud, map->getMutable());

      LOG(INFO) << "Updating orthomosaic layer with " << T_G_Bs_subset.size()
                << " image-pose-pairs";
      mosaic->process(T_G_Bs_subset, images_subset, map->getMutable());

      LOG(INFO) << "Publishing";
      map->publishOnce();
      map->publishImageOnce();
      images_subset.clear();
      T_G_Bs_subset.clear();
    }
    ++pcl_cnt;
    last_pos = pos;
  }
  //LOG(INFO) << "Cb complete";
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  last_pos.setIdentity();

  ros::init(argc, argv, "ortho_backward_grid_incremental");
  ros::Time::init();
  ros::NodeHandle nh("~");

  // Parse input parameters.
  const std::string& base = FLAGS_backward_grid_data_directory;
  const std::string& filename_camera_rig =
      FLAGS_backward_grid_filename_camera_rig;
  const std::string& filename_poses = FLAGS_backward_grid_filename_poses;
  const std::string& filename_images = base + FLAGS_backward_grid_prefix_images;

  // Load camera rig from file.
  io::AerialMapperIO io_handler;
  const std::string& filename_camera_rig_yaml = base + filename_camera_rig;
  std::shared_ptr<aslam::NCamera> ncameras =
      io_handler.loadCameraRigFromFile(filename_camera_rig_yaml);
  CHECK(ncameras);

  // Load body poses from file.
  //Poses T_G_Bs;
  //const std::string& path_filename_poses = base + filename_poses;
  //io::PoseFormat pose_format = io::PoseFormat::Standard;
  //io_handler.loadPosesFromFile(pose_format, path_filename_poses, &T_G_Bs);

  //// Load images from file.
  //size_t num_poses = T_G_Bs.size();
  //Images images;
  //io_handler.loadImagesFromFile(filename_images, num_poses, &images,
  //                              FLAGS_backward_grid_colored_ortho);

  // Set up layered map (grid_map).
  grid_map::Settings settings_aerial_grid_map;
  settings_aerial_grid_map.center_easting = FLAGS_backward_grid_center_easting;
  settings_aerial_grid_map.center_northing =
      FLAGS_backward_grid_center_northing;
  settings_aerial_grid_map.delta_easting = FLAGS_backward_grid_delta_easting;
  settings_aerial_grid_map.delta_northing = FLAGS_backward_grid_delta_northing;
  settings_aerial_grid_map.resolution = FLAGS_backward_grid_resolution;
  map = new grid_map::AerialGridMap(settings_aerial_grid_map);

  // Set up dense reconstruction.
  stereo::Settings settings_dense_pcl;
  settings_dense_pcl.use_every_nth_image = FLAGS_dense_pcl_use_every_nth_image;
  settings_dense_pcl.show_rectification = false;
  LOG(INFO) << "Perform dense reconstruction using planar rectification.";
  stereo::BlockMatchingParameters block_matching_params;
  block_matching_params.use_BM = FLAGS_use_BM;
  stereo_proc = new stereo::Stereo(ncameras, settings_dense_pcl, block_matching_params);

  // Set up digital surface map.
  dsm::Settings settings_dsm;
  settings_dsm.center_easting = settings_aerial_grid_map.center_easting;
  settings_dsm.center_northing = settings_aerial_grid_map.center_northing;
  digital_surface_map = new dsm::Dsm(settings_dsm, map->getMutable());

  // Set up orthomosaic.
  ortho::Settings settings_ortho;
  settings_ortho.show_orthomosaic_opencv =
      FLAGS_backward_grid_show_orthomosaic_opencv;
  settings_ortho.save_orthomosaic_jpg =
      FLAGS_backward_grid_save_orthomosaic_jpg;
  settings_ortho.orthomosaic_jpg_filename =
      FLAGS_backward_grid_orthomosaic_jpg_filename;
  settings_ortho.orthomosaic_elevation_m =
      FLAGS_backward_grid_orthomosaic_elevation_m;
  settings_ortho.use_digital_elevation_map =
      FLAGS_backward_grid_use_digital_elevation_map;
  settings_ortho.colored_ortho = FLAGS_backward_grid_colored_ortho;
  settings_ortho.use_multi_threads = FLAGS_backward_grid_use_multi_threads;
  mosaic = new ortho::OrthoBackwardGrid(ncameras, settings_ortho, map->getMutable());

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 100);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "pose", 100);
  message_filters::Subscriber<sensor_msgs::Image> image_sem_sub(nh, "image_sem", 100);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> *sem_sync;
  message_filters::TimeSynchronizer<sensor_msgs::Image, geometry_msgs::PoseStamped> *sync;
  if (FLAGS_integrate_semantics) {
    sem_sync = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped>
      (image_sub, image_sem_sub, pose_sub, 100);
    sem_sync->registerCallback(boost::bind(&imgCb, _1, _2, _3));
  } else {
    sync = new message_filters::TimeSynchronizer<sensor_msgs::Image, geometry_msgs::PoseStamped>
      (image_sub, pose_sub, 100);
    sync->registerCallback(boost::bind(&imgCb, _1, nullptr, _2));
  }

  ROS_INFO_STREAM("config complete");
  ros::spin();

  return 0;
}
