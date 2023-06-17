/**
  MIT License

  Copyright (c) 2023 Hei Yin Wong, Agung Nuza Dwiputra, Akshet Patel

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 **/

/**
 * @file cw3_class.h
 * @author Hei Yin Wong, Agung Nuza Dwiputra, Akshet Patel
 * @date 14.04.2023
 * @brief header file for the cw3 class
 * @defgroup cw3_class CW3 Class
 */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>

// // include any services created in this package
//For task2 
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// OpenCV Includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <cstdlib>
#include <cmath>
#include <Eigen/Dense>

// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"
#include "target_shape.h"
/**
 * @brief Typedef for the PointXYZRGBA type in PCL.
 */
typedef pcl::PointXYZRGBA PointT;

/**
  * @brief Typedef for a PointCloud of PointXYZRGBA points.
  */
typedef pcl::PointCloud<PointT> PointC;

/**
  * @brief Typedef for a shared pointer to a PointCloud of PointXYZRGBA points.
  */
typedef PointC::Ptr PointCPtr;

class cw3
{
public:

  /* ----- class member functions ----- */

  // constructor
  /**
   * @brief Constructor for the class.
   * @param nh The ROS node handle.
   */
  cw3(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3

  /**
   * @brief Service callback function for task 1.
   * @param request The request data for the service call.
   * @param response The response data for the service call.
   * @return True if the service call is successful, false otherwise.
   */
  bool 
  t1_callback(cw3_world_spawner::Task1Service::Request &request,
    cw3_world_spawner::Task1Service::Response &response);

  /**
   * @brief Service callback function for task 2.
   * @param request The request data for the service call.
   * @param response The response data for the service call.
   * @return True if the service call is successful, false otherwise.
   */
  bool 
  t2_callback(cw3_world_spawner::Task2Service::Request &request,
    cw3_world_spawner::Task2Service::Response &response);

  /**
   * @brief Service callback function for task 3.
   * @param request The request data for the service call.
   * @param response The response data for the service call.
   * @return True if the service call is successful, false otherwise.
   */
  bool 
  t3_callback(cw3_world_spawner::Task3Service::Request &request,
    cw3_world_spawner::Task3Service::Response &response);

  /* ----- class member variables ----- */

  /**
   * @brief ROS node handle
   */
  ros::NodeHandle nh_;

  /**
   * @brief ROS service server for task 1
   */
  ros::ServiceServer t1_service_;

  /**
   * @brief ROS service server for task 2
   */
  ros::ServiceServer t2_service_;

  /**
   * @brief ROS service server for task 3
   */
  ros::ServiceServer t3_service_;

  /**
   * @brief ROS publisher for publishing point clouds
   */
  ros::Publisher g_pub_cloud_;
  
 /**
   * @brief ROS subscriber for RGB camera topic
   */
  ros::Subscriber rgb_camera_subscriber_;  

  ros::Subscriber depth_registered_point_subscriber_;

  /** @brief MoveIt interface to move groups to seperate the arm and the 
   * gripper,
   * these are defined in urdf. 
   */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

private:

  //Reset robot to the home (initial) position
  /**
   * @brief Function to reset the robot to its home (initial) position
   *
   * The function sends commands to robot to reset its position and orientation
   * to its home (initial) position
   *
   */
  void homingRobot();

  /**
   * @brief Function to move the gripper to the specified width
   *
   * The function sends a command to the gripper to move to the specified width
   *
   * @param[in] width The desired width of the gripper
   *
   * @return true if the gripper successfully moves to the specified width, else 
   * false
   */
  bool moveGripper(float width);

  bool
  moveArm(geometry_msgs::Pose target_pose);

  /**
   * @brief Moves the robot to the specified linear position (x,y,z)
   * @param x The x position to move to
   * @param y The y position to move to
   * @param z The z position to move to
   * @return True if the robot successfully moved to the target position, 
   * false otherwise
   */
  bool moveRobotLinear(double x, double y, double z);

  /**
   * @brief callback function for the RGB camera topic
   * @param[in] msg The message data of the RGB image
   */
  void
  rgbCameraCallback(const sensor_msgs::Image &msg);

  /**
   * @brief Callback function for the depth registered point cloud input.
   * This function is called whenever a new depth registered point cloud message
   * is received. The function applies several filters to the point cloud data,
   * including downsampling, removal of the green floor, and keeping only the
   * red and blue objects.
   * @param cloud_input_msg The input point cloud message.
   */
  void depthRegisteredPointCallback(
      const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  /**
   * @brief Apply point transformation on the input cloud
   *
   * This function takes an input cloud and applies point transformation on it
   *
   * @param[in] in_cloud_ptr  The input cloud for the transformation
   * @param[out] out_cloud_ptr  The transformed output cloud 
   */
  void
  applyPT (PointCPtr &in_cloud_ptr,PointCPtr &out_cloud_ptr);

  /**
   * @brief Function to execute a robot trajectory
   *
   * The function sends commands to the robot to execute a trajectory defined by 
   * a series of waypoints.
   *
   * @param[in] waypoints A vector of geometry_msgs::Pose objects representing 
   * the waypoints of the trajectory
   *
   * @return true if the robot successfully executes the trajectory, else flase
   */
  bool 
  executeRobotTrajectory(std::vector<geometry_msgs::Pose> waypoints);


  /**
   * @brief Publishes a PointCloud message after filtering
   *
   * The function takes in a PointCloud message, filters it, and publishes the 
   * filtered message to a specified topic.
   *
   * @param[in,out] pc_pub The ROS Publisher object used to publish the message
   * @param[in] pc The PointCloud message to be filtered and published
   */
  void
  pubFilteredPCMsg (ros::Publisher &pc_pub,PointC &pc);

  /**
   * @brief Flag indicating whether the camera is being checked or not.
   */
  bool is_checking_camera = true;

  /**
   * @brief Detects lines in an image using a Hough transform and saves 
   * them to a file.
   */
  void
  detectLines();

  /**
   * @brief Detects landmarks in a point cloud and returns the number of 
   * landmarks detected.
   * @return The number of landmarks detected.
   */
  int
  detectLandmarks();
  
  /**
   * @brief Detects lines in an image with orientation using a Hough transform 
   * and returns the orientation.
   * @return The orientation detected.
   */
  double
  detectLinesWithOrientation();

  /**
   * @brief Scans for a shape in a given zone.
   * @param x The x-coordinate of the zone.
   * @param y The y-coordinate of the zone.
   * @param zone The zone to scan for the shape.
   */
  void 
  scanShape(double x, double y,int zone);

  /**
   * @brief Finds the accurate location of a shape's centroid in the world 
   * frame for a given zone.
   * @param centroid_in_world_frame The accurate location of the centroid of 
   * the shape in the world frame.
   * @param zone The zone in which the shape is located.
   */
  void 
  findShapeAcurateLocation(geometry_msgs::PointStamped& 
  centroid_in_world_frame,int zone);

  /**
   * @brief Finds the location of the basket.
   */
  void
  findBasket();

  /**
   * @brief Pointer to a CvImage object.
   */
  cv_bridge::CvImagePtr cv_ptr;

  /**
   * @brief Moves the robot linearly with rotation.
   * @param x The x position to move to.
   * @param y The y position to move to.
   * @param z The z position to move to.
   * @param yaw The yaw angle to rotate to.
   * @return True if the movement was successful, false otherwise.
   */
  bool
  moveRobotLinearWithRotation(double x,double y,double z, double yaw);

  /* ----- class member variables ----- */  
  
  /** 
   * @brief Point Cloud (input) pointer. 
   */
  PointCPtr g_cloud_ptr_;
  
  /** 
   * @brief Point Cloud (filtered) pointer. 
   */
  PointCPtr g_cloud_filtered_;

  /** 
  * @brief Point Cloud (filtered) sensros_msg for publ. 
  */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg_;

  /** 
   * @brief Voxel Grid filter. 
   */
  pcl::VoxelGrid<PointT> g_vx_;

  /** 
   * @brief Point Cloud (input). 
   */
  pcl::PCLPointCloud2 g_pcl_pc_;

  /** 
   * @brief The input point cloud frame id. 
   */
  std::string g_input_pc_frame_id_;

  /**
   * @brief Mutex for synchronizing access to shared data.
   */
  std::mutex mutex_pcl_;

  /**
   * @brief The offset in the x direction of the camera position.
   */
  const double CAMERA_OFFSET_X = -0.04234; 

  /**
   * @brief The offset in the y direction of the camera position.
   */
  const double CAMERA_OFFSET_Y = 0.00018; 
  
  /**
   * @brief Define the maximum open value for the gripper.
   */
  const double GRIPPER_MAX_OPEN = 80e-3;

  /**
   * @brief Define the closed value for the gripper.
   */
  const double GRIPPER_CLOSED = 0.0;

  /**
   * @brief The width of the image in pixels.
   */
  const double IMAGE_WIDTH = 640;
  
  /**
   * @brief The center x-coordinate of the image in pixels.
   */
  const double IMAGE_CENTER_X = 319;

  /**
   * @brief The center y-coordinate of the image in pixels.
   */
  const double IMAGE_CENTER_Y = 239;

  /**
   * @brief Define the x orientation for picking up an object.
   */
  const double PICK_ORIENTATION_X = 0.9249091;

  /**
   * @brief Define the y orientation for picking up an object.
   */
  const double PICK_ORIENTATION_Y = -0.3801884;

  /**
   * @brief Define the z orientation for picking up an object.
   */
  const double PICK_ORIENTATION_Z = 0.0000176;  

  /**
   * @brief Define the w orientation for picking up an object.
   */
  const double PICK_ORIENTATION_W = 0.0000428;

  /**
   * @brief Define the red threshold for object detection.
   */
  const uint8_t RED_THRESHOLD = 100;  

  /**
   * @brief Define the blue threshold for object detection.
   */
  const uint8_t BLUE_THRESHOLD = 100;

  /**
   * @brief The size of the leaf in the voxel grid filter.
   */
  // const double G_VG_LEAF_SZ = 0.001;
  const double G_VG_LEAF_SZ = 0.0025;
  
  // std::vector<geometry_msgs::Pose> shape_pos_map;
  std::vector<TargetShape> shape_pos_map_;

    /** 
   * @brief The frame id used for the Point Cloud data. 
   */
  std::string FRAME_ID = "color";

  /** 
   * @brief The frame id for the robot's link 0. 
   */
  std::string PANDA_FRAME_0 = "panda_link0";

    /** 
   * @brief A transform listener for the robot. 
   */
  tf::TransformListener g_listener_;

  /**
   * @brief Uses computer vision techniques to classify the shape of a target.
   * @return A string representing the shape of the target.
   */
  std::string
  classifyShapeCV();
  
  /**
   * @brief Define the location of the 4 baskets in the environment. 
   */
  const double BASKET_LOC[2][2]= {{-0.41,-0.36},{-0.41,0.36}};

  /**
   * @brief Index of the basket where the shapes will be placed.
   */
  int basket_idx_ = -1; //will be updated by find basket

  /**
   * @brief Moves the nought shape to the basket located at position (x,y).
   * @param x X coordinate of the basket.
   * @param y Y coordinate of the basket.
   * @param goal_point 3D coordinates of the basket.
   */
  void
  moveNoughtToBasket(double x, double y,geometry_msgs::Point goal_point);

  /**
   * @brief Moves the cross shape to the basket located at position (x,y).
   * @param x X coordinate of the basket.
   * @param y Y coordinate of the basket.
   * @param goal_point 3D coordinates of the basket.
   */
  void 
  moveCrossToBasket(double x, double y,geometry_msgs::Point goal_point);

};

#endif // end of include guard for cw3_CLASS_H_