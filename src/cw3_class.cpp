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

////////////////////////////////////////////////////////////////////////////////
/////IF YOU ARE A RECRUITER WHO NEEDS TO RUN THIS CODE, PLEASE EMAIL ME AT//////
////////////////////////////akshetp.ap@gmail.com ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
#include <cw3_class.h> // change to your team name here!

/**
 * Constructor for the `cw3` class.
 *
 * This constructor initializes the member variables and services for the 
 * `cw1` class.
 *
 * @param nh A ROS node handle.
 */
cw3::cw3(ros::NodeHandle nh)
{
  /* class constructor */

  nh_ = nh;

  // //Init member variables  
  g_cloud_ptr_ = PointC::Ptr(new PointC);
  g_cloud_filtered_ = PointC::Ptr(new PointC);

  //Allow a reasonable tolerance, otherwise the robot might spent too much time 
  // no solution for computing the cartesian path
  arm_group_.setGoalPositionTolerance(0.001);

  //Home the robot before taking service requests
  homingRobot();


  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw3::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw3::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw3::t3_callback, this);


  //Define the publisher for the filtered point cloud
  g_pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, 
  true);

  //Subscribe to the camera topic
  rgb_camera_subscriber_ = nh_.subscribe("/r200/camera/color/image_raw", 1, 
  &cw3::rgbCameraCallback, this);

  depth_registered_point_subscriber_ = 
  nh_.subscribe("/r200/camera/depth_registered/points", 1, 
  &cw3::depthRegisteredPointCallback, this);

  ROS_INFO("cw3 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

/**
 * This is the callback function for solving Task 1 of the coursework.
 * When the service is triggered, it will first pick the shape from the object 
 * location specified in the request and then place it at the goal location also
 * specified in the request.
 * Finally, it will home the robot.
 * 
 * @param[in] request contains the information about the object location and 
 * goal location
 * @param[out] response contains the outcome of the task, whether it was 
 * successful or not
 * 
 * @return returns true if the task is successful
 */
bool
cw3::t1_callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  // Set flag to indicate that camera is being checked
  is_checking_camera = true;

  // Print info message to indicate that callback has been triggered
  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // Extract coordinates of object (shape) point from request message
  double x = request.object_point.point.x;
  double y = request.object_point.point.y;
  double z = request.object_point.point.z;
  
  // Extract shape type from request message
  std::string shape_type = request.shape_type;

  // Print info message to show the coordinates of the pick point and shape type
  ROS_INFO("T1 received pick point: (%f, %f, %f)", x, y, z);
  ROS_INFO("Shape type: %s", shape_type.c_str());

  // Set flag to indicate that camera is being checked
  is_checking_camera = true;

  // Move the robot linearly to the object point with an offset for the camera
  moveRobotLinear(x+CAMERA_OFFSET_X ,y+CAMERA_OFFSET_Y,z+0.37);

  // Detect the orientation angle of the object using an image and save it to 
  // theta variable
  double theta = detectLinesWithOrientation();

  // Print the orientation angle
  ROS_INFO("==theta: %f", theta);

  // If the shape type is "nought"
  if(shape_type == "nought")
  {
    // Set Quaternion q to deal with the rotation
    tf2::Quaternion q;
    q.setRPY(0,0,-theta);  

    // Set the offset for the gripper position
    tf2::Vector3 pos = tf2::Vector3(0,-0.08,0);

    // Rotate the offset by the orientation angle, theta
    tf2::Vector3 pos_rotated = tf2::quatRotate(q, pos);
    
    // Move the robot linearly to the object point with the gripper offset and 
    // the orientation angle
    moveRobotLinearWithRotation
    (x+pos_rotated.x() ,y+pos_rotated.y(),z+0.37,theta);   

    // Open the gripper
    moveGripper(GRIPPER_MAX_OPEN);    

    // Move the robot linearly down to the object with the appropriate rotation
    moveRobotLinearWithRotation
    (x+pos_rotated.x() ,y+pos_rotated.y(),z+0.15,theta);

    // Close the gripper
    moveGripper(GRIPPER_CLOSED);

    // Move the robot linearly to the goal point
    moveRobotLinearWithRotation
    (x+pos_rotated.x() ,y+pos_rotated.y(),z+0.37,theta);
    moveRobotLinear(x  ,request.goal_point.point.y+0.08,z+0.37);
    moveRobotLinear
    (request.goal_point.point.x,request.goal_point.point.y-0.08,z+0.5); 

    // Move the robot linearly down to the object drop point
    moveRobotLinear
    (request.goal_point.point.x ,request.goal_point.point.y-0.08,z+0.2); 

    // Open the gripper to drop the object
    moveGripper(GRIPPER_MAX_OPEN);

    // Move the robot linearly back up and go to object position
    moveRobotLinear
    (request.goal_point.point.x,request.goal_point.point.y,z+0.37);
    moveRobotLinear(x  ,request.goal_point.point.y,z+0.37); 
    moveRobotLinear(x ,y,z+0.37);

    // Home the robot
    homingRobot();

  } 
  // if the shape of the object is "cross"
  else if (shape_type == "cross")
  {
    // Set Quaternion q to deal with the rotation
    tf2::Quaternion q;
    q.setRPY(0,0,-theta);  

    // Set the offset for the gripper position
    tf2::Vector3 pos = tf2::Vector3(-0.04,0,0);   

    // Rotate the offset by the orientation angle, theta
    tf2::Vector3 pos_rotated = tf2::quatRotate(q, pos);

    // Move the robot linearly to the object point with the gripper offset and 
    // the orientation angle
    moveRobotLinearWithRotation
    (x+pos_rotated.x() ,y+pos_rotated.y(),z+0.37,theta); 

    // Open the gripper
    moveGripper(GRIPPER_MAX_OPEN);    

    // Move the robot linearly down to the object with the appropriate rotation
    moveRobotLinearWithRotation
    (x+pos_rotated.x() ,y+pos_rotated.y(),z+0.15,theta); 

    // Close the gripper
    moveGripper(GRIPPER_CLOSED);

    // Move the robot linearly to the goal point
    moveRobotLinearWithRotation
    (x+pos_rotated.x() ,y+pos_rotated.y(),z+0.37,theta); 
    moveRobotLinear(x  ,request.goal_point.point.y,z+0.37); 
    moveRobotLinear
    (request.goal_point.point.x-0.04,request.goal_point.point.y,z+0.5); 

    // Move the robot linearly down to the object drop point
    moveRobotLinear
    (request.goal_point.point.x-0.04 ,request.goal_point.point.y,z+0.2); 

    // Open the gripper to drop the object
    moveGripper(GRIPPER_MAX_OPEN);

    // Move the robot linearly back up and go to object position
    moveRobotLinear
    (request.goal_point.point.x,request.goal_point.point.y,z+0.37); 
    moveRobotLinear(x  ,request.goal_point.point.y,z+0.37);
    moveRobotLinear(x ,y,z+0.37);

    // Home the robot
    homingRobot();
    
  }
  // if shape is not recognised, print error message
  else
  {
    ROS_INFO("Shape type not recognised");
  }
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Callback function for solving task 2.
 * 
 * This function retrieves a request object containing reference object points
 * and a mystery object point. It performs necessary actions to determine the
 * shape of the mystery object by comparing the detected number of landmarks in
 * each reference object with number of landmarks detected in the mystery object
 * and then stores the shape of the mystery object in the response object.
 * 
 * @param request A request object containing reference object points and a 
 * mystery object point.
 * @param response A response object that will store the shape of 
 * the mystery object.
 * @return A boolean indicating whether the task was solved successfully.
 */
bool
cw3::t2_callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  // Print info message to indicate that callback has been triggered
  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  // Set flag to indicate that camera is being checked
  is_checking_camera = true;

  // Create a vector to store landmark count cache
  std::vector<int> landmark_count_cache;


  // Parse request and move robot to each reference object
  for (int i = 0 ; i < request.ref_object_points.size(); i++)
  {  
    double x = request.ref_object_points[i].point.x;
    double y = request.ref_object_points[i].point.y;
    double z = request.ref_object_points[i].point.z;

    // Print reference object pick point coordinates
    ROS_INFO("T2 received pick point: (%f, %f, %f)", x, y, z);
    
    // Move robot to the reference object
    moveRobotLinear(0.3  , y , z+0.37); 
    moveRobotLinear(x + CAMERA_OFFSET_X  , y + CAMERA_OFFSET_Y, z+0.37); 

    // Wait for camera to stabilize and detect the number of landmarks
    ros::Duration(2).sleep();
    landmark_count_cache.push_back(detectLandmarks());

    // Return robot to the pick point
    moveRobotLinear(0.3  , y , z+0.37); 

    // Home the robot
    homingRobot();
  }

  // Move robot to the mystery object and detect its shape
  double x = request.mystery_object_point.point.x;
  double y = request.mystery_object_point.point.y;
  double z = request.mystery_object_point.point.z;
  moveRobotLinear(x + CAMERA_OFFSET_X , y+CAMERA_OFFSET_Y , z+0.37);
  ros::Duration(2).sleep();
  int mystery_shape = detectLandmarks();

  homingRobot();

  // Print the detected mystery object shape and the shapes of reference objects
  ROS_INFO("=================================");
  ROS_INFO("Mystery shape: %d", mystery_shape);

  for (int i = 0 ; i < landmark_count_cache.size(); i++)
  {
    ROS_INFO("Shape: %d", landmark_count_cache[i]);
  }

  // Compare the number of detected landmarks in the mystery object 
  // with the reference objects
  int shape_type  =  (mystery_shape == landmark_count_cache[0])?1:2;
  
  // Print the detected shape type and store it in the response object
  ROS_INFO("Return DETECTED Shape type: %d", shape_type);
  response.mystery_object_num = shape_type;

  // Clear landmark count cache and return success
  landmark_count_cache.clear();
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Solves task 3 of the coursework.
 * This function solves task 3 of the coursework by performing a series of 
 * actions with the robot, including moving to different locations, scanning 
 * for shapes, and picking up objects. The function first clears the 
 * shape_pos_map_, then homes the robot and moves it to a specific joint state 
 * target. After that, it plans and executes a series of movements to scan for 
 * and pick up objects in different areas of the environment. Finally, it homes 
 * the robot again and computes various statistics about the shapes it found.
 * @param request The request object passed to the service callback.
 * @param response The response object passed to the service callback.
 * @return True if the task was completed successfully, false otherwise.
 */
bool
cw3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  //////////////////////////////////////////////////////////////////////////////
  ///////WE HAVE EXPLAINED THE STEPS PERFORMED FOR TASK 3 IN THE REPORT/////////
  //////////////////////////////////////////////////////////////////////////////

  /* function which should solve task 3 */
  // Clear the map of shape positions
  shape_pos_map_.clear();
  
  // Print info message to indicate that callback has been triggered
  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  // Home the robot
  homingRobot();

  // Get the current joint state of the robot arm
  std::vector<double> joint_state_target = arm_group_.getCurrentJointValues();
  // Set the home state target to be the same as the current joint state
  std::vector<double> home_state_target = arm_group_.getCurrentJointValues();
  
  // Set the joint state target for the first joint to -178 degrees
  joint_state_target[0] = - 178 * M_PI / 180;  
  
  // Set the joint value target for the robot arm
  arm_group_.setJointValueTarget(joint_state_target);
  
  // Set the velocity and acceleration scaling factors to increase speed
  arm_group_.setMaxVelocityScalingFactor(1);
  arm_group_.setMaxAccelerationScalingFactor(1);
  

  // Attempt to plan the path of the robot arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Print a message to indicate whether the planning was successful or not
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // Move the robot arm along the planned path
  arm_group_.move();
  // Print the joint 1 value of the robot arm
  ROS_INFO("Joint 1: %f", joint_state_target[0]);
  
  // Get the current pose of the robot arm
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  // Print the current position of the robot arm
  ROS_INFO("Current pose: %f, %f, %f", current_pose.pose.position.x, 
  current_pose.pose.position.y, current_pose.pose.position.z);

  // Scanning shapes from Top
  scanShape(-0.25, -0.01,1);  //Top
  scanShape(-0.45, -0.01,2);  //Top  

  scanShape(-0.4,-0.45,3); //Top left

  // Move robot linearly to find basket
  moveRobotLinear(-0.4,-0.36,0.6); //find basket
  
  // Find the basket
  findBasket();

  // Scanning shapes from Top Left
  scanShape(-0.2,-0.45,4); //Top left

  scanShape(0,-0.45,5); //Mid left 

  scanShape(0.3,-0.45,6); //Front left
  scanShape(0.6,-0.45,7); //Front left

  scanShape(0.3,-0.15,8); //Front
  scanShape(0.6,-0.15,9); //Front

  scanShape(0.3,0.15,8); //Front
  scanShape(0.6,0.15,9); //Front

  // Scanning shapes from front right
  scanShape(0.6,0.45,10); //Front right
  scanShape(0.3,0.45,11); //Front right

  // Move joint to 90 degrees and scan shapes from mid right
  joint_state_target[0] = 90 * M_PI / 180;
  arm_group_.setJointValueTarget(joint_state_target);
  arm_group_.move();

  scanShape(-0.01,0.45,12); //Mid right
  scanShape(-0.25,0.45,13); //Mid right

  // Move joint to 140 degrees and scan shapes from top right
  joint_state_target[0] = 140 * M_PI / 180;
  arm_group_.setJointValueTarget(joint_state_target);
  arm_group_.move();
  
  // Get the current pose of the robot end-effector
  geometry_msgs::PoseStamped current_pose2 = arm_group_.getCurrentPose();
  // Print the position of the robot end-effector
  ROS_INFO("Current pose: %f, %f, %f", current_pose2.pose.position.x, 
  current_pose2.pose.position.y, current_pose2.pose.position.z);
  // Print the orientation of the robot end-effector using Roll-Pitch-Yaw angles
  ROS_INFO("RPY: %f, %f, %f ,%f", current_pose2.pose.orientation.x, 
  current_pose2.pose.orientation.y, current_pose2.pose.orientation.z, 
  current_pose2.pose.orientation.w);

  scanShape(-0.45,0.45,14); //Top right

  // Set the target joint state to move the robot arm to a homing position
  joint_state_target[0] = 140 * M_PI / 180;
  arm_group_.setJointValueTarget(joint_state_target);
  // Move the robot arm to the homing position
  arm_group_.move();
  
  // Set the target joint state to move the robot arm to a vertical position
  joint_state_target[0] = 0;
  arm_group_.setJointValueTarget(joint_state_target);
  // Move the robot arm to the vertical position
  arm_group_.move();

  // Home the robot
  homingRobot();
  
  // Initialize variables for counting the number of shapes and the number of 
  // noughts and crosses
  int total_shapes = 0;
  int noughts_count = 0;
  int crosses_count = 0;
  int noughts_idx = 0 ;
  int crosses_idx = 0;

  //Pick the object to basket
  // Iterate through each target shape in the shape_pos_map_ and print output
  ROS_INFO("Shape pos map size: %ld", shape_pos_map_.size());
  for (int i = 0; i < this->shape_pos_map_.size(); i++)
  {
    TargetShape b = this->shape_pos_map_.at(i);

    ROS_INFO("Shape: %s - size: %d - POS: x: %f, y: %f, z: %f, zone: %d", 
    b.getShapeType().c_str(),b.getNumOfPoints(),b.getX(), b.getY(), b.getZ(),
    b.getZone());
     // If the current target shape is a nought, increment the noughts count and 
     // store its index
     if(b.getShapeType()=="Nought")
     {
      //Pick the first one
      //check size / z
      noughts_idx = (noughts_idx==0)?i:noughts_idx;
      noughts_count++;
    }
    // If the current target shape is a cross, increment the crosses count and 
    // store its index
    else if(b.getShapeType()=="Cross")
    {
      //Pick the first one
      //check size / z
      crosses_idx = (crosses_idx==0)?i:crosses_idx;
      crosses_count++;
    }    
  }

  // Set the target basket location based on the current basket index
  geometry_msgs::Point p; 
  p.x = BASKET_LOC[basket_idx_][0];
  p.y = BASKET_LOC[basket_idx_][1];

  // If there are more crosses than noughts, move the first cross to the basket
  if(crosses_count >= noughts_count)
  {
    TargetShape b = this->shape_pos_map_.at(crosses_idx);
    moveCrossToBasket(b.getX(),b.getY(),p);
  }
  // If there are more noughts than crosses, move the first nought to the basket
  else
  {
    TargetShape b = this->shape_pos_map_.at(noughts_idx);
    moveNoughtToBasket(b.getX(),b.getY(),p);
  }
   
  

  //print shape_pos_map_
  // Iterate through each target shape in the shape_pos_map_ and print output
  ROS_INFO("Shape pos map size: %ld", shape_pos_map_.size());
  for (int i = 0; i < this->shape_pos_map_.size(); i++)
  {
    TargetShape b = this->shape_pos_map_.at(i);

    // Print shape type, number of points, position, and zone
    ROS_INFO("Shape: %s - size: %d - POS: x: %f, y: %f, z: %f, zone: %d", 
    b.getShapeType().c_str(),b.getNumOfPoints(),b.getX(), b.getY(), b.getZ(),
    b.getZone());

  }
  // Print the basket location
  ROS_INFO("Basket location: x: %f, y: %f", BASKET_LOC[basket_idx_][0], 
  BASKET_LOC[basket_idx_][1]);

  // Home the robot
  homingRobot();

  arm_group_.setJointValueTarget(home_state_target);
  arm_group_.move();

  // Construct the reply   
  response.total_num_shapes = noughts_count + crosses_count;
  response.num_most_common_shape = noughts_count > crosses_count ? noughts_count
   : crosses_count;

  // Return the success response
  return true;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Callback function for the RGB camera.
 * 
 * This function is called when a new RGB image is received. If the flag 
 * is_checking_camera is set to true, the function extracts image dimensions
 * and allocates memory for image data. It then copies the image data from the 
 * ROS message to an OpenCV matrix using memcpy.
 * If successful, it creates a new cv_bridge object and stores the 
 * OpenCV matrix in it.
 * 
 * @param msg The ROS message containing the RGB image data.
 */
void
cw3::rgbCameraCallback(const sensor_msgs::Image &msg)
{

  if (is_checking_camera)
  {
    // Extract image dimensions and step size
    const int width = msg.width;
    const int height = msg.height;
    const int step = msg.step;

    // Allocate memory for image data
    cv::Mat image(height, width, CV_8UC3);

    // Copy image data from ROS message to OpenCV matrix
    const uint8_t* src = &msg.data[0];
    uint8_t* dst = image.data;

    for (int y = 0; y < height; y++) 
    {
      std::memcpy(dst, src, width*3);
      src += step;
      dst += width*3;
    }

    // Create new cv_bridge object and store OpenCV matrix in it
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }    
  }
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Detects lines in an image using Canny edge detection and 
 * Hough line transform.
 */
void 
cw3::detectLines()
{
  cv::Mat src_gray;
  cv::cvtColor(cv_ptr->image, src_gray, cv::COLOR_BGR2GRAY);

  // Apply Canny edge detection
  cv::Mat edges;
  cv::Canny(src_gray, edges, 50, 200, 3);

  // Perform Hough Line Transform
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(edges, lines, 1, CV_PI/180, 106, 0, 0);

  // Draw lines on the image
  cv::Mat linesImage = cv_ptr->image.clone();

  // Set the region of interest (ROI) to the entire image
  cv::Rect roi(0, 0, linesImage.cols, linesImage.rows); 

  // Vector to store the midpoints
  std::vector<cv::Point> midpoints;

  for (size_t i = 0; i < lines.size(); i++) 
  {
    // Extract rho and theta values from the current line
    float rho = lines[i][0], theta = lines[i][1];

    // Define two points for the line based on rho and theta
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));

    // Print the points of the line
    ROS_INFO("Line %zu: (%d,%d) - (%d,%d)", i, pt1.x, pt1.y, pt2.x, pt2.y);

    // Clip the line to the ROI
    cv::clipLine(roi, pt1, pt2);

    // Draw the line only if it intersects the ROI
    if (pt1 != pt2)
    {
      cv::line(linesImage, pt1, pt2, cv::Scalar(0,0,0), 2, cv::LINE_AA);

      // Midpoint of the lines
      cv::Point midpoint((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2);

      // Add the midpoint to the vector
      midpoints.push_back(midpoint); 

      // Draw midpoint in red
      cv::circle(linesImage, midpoint, 3, cv::Scalar(0, 0, 255), -1);

      // Print the midpoint
      ROS_INFO("Midpoint of line %zu: (%d, %d)", i, midpoint.x, midpoint.y);
      
    }
}

double res_x = 0;
double res_y = 0; 

for (size_t i = 0; i < midpoints.size(); i++)
{
  for (size_t j = i+1; j < midpoints.size(); j++)
  {
    // Calculate the distance between the midpoints
    double distance = cv::norm(midpoints[i] - midpoints[j]);

    // check if the distance between two midpoints is in between 30 and 120
    if (distance > 30 && distance < 120)
    {
      // Draw a line between the midpoints
      cv::line(linesImage, midpoints[i], midpoints[j], 
      cv::Scalar(0,0,0), 2, cv::LINE_AA);

      // Calculate and draw the midpoint of the line joining the midpoints
      cv::Point midpoint_line((midpoints[i].x + midpoints[j].x) / 2, 
      (midpoints[i].y + midpoints[j].y) / 2);
      cv::circle(linesImage, midpoint_line, 3, cv::Scalar(255, 255, 255), -1);

      // Print appropriate messages of the distance and the midpoints
      ROS_INFO("Distance between midpoints %zu and %zu: %f", i, j, distance);
      ROS_INFO("Midpoint %zu: (%d, %d)", i, midpoints[i].x, midpoints[i].y);
      ROS_INFO("Midpoint %zu: (%d, %d)", j, midpoints[j].x, midpoints[j].y);
      ROS_INFO("Midpoint of line joining midpoints %zu and %zu: (%d, %d)", 
      i, j, midpoint_line.x, midpoint_line.y);
    }
  }
}

// print the number of lines
ROS_INFO("Number of lines detected: %lu", lines.size());

}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Detects landmarks in an image using Canny edge detection and 
 * Hough line transform
 * 
 * @return int Returns 1 if a cross is detected, 0 otherwise
 */
int 
cw3::detectLandmarks()
{
  cv::Mat src_gray;
  cv::cvtColor(cv_ptr->image, src_gray, cv::COLOR_BGR2GRAY);

  // Apply Canny edge detection
  cv::Mat edges;
  cv::Canny(src_gray, edges, 50, 200, 3);

  // Perform Hough Line Transform
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(edges, lines, 1, CV_PI/180, 106, 0, 0);

  // Draw lines on the image
  cv::Mat linesImage = cv_ptr->image.clone();

  // Set the region of interest (ROI) to the entire image
  cv::Rect roi(0, 0, linesImage.cols, linesImage.rows); 

  // Vector to store the midpoints
  std::vector<cv::Point> midpoints;

  for (size_t i = 0; i < lines.size(); i++) 
  {
    // Extract rho and theta values from the current line
    float rho = lines[i][0], theta = lines[i][1];

    // Define two points for the line based on rho and theta
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));

    // Print the points of the line
    ROS_INFO("Line %zu: (%d,%d) - (%d,%d)", i, pt1.x, pt1.y, pt2.x, pt2.y);

    // Clip the line to the ROI
    cv::clipLine(roi, pt1, pt2);

    // Draw the line only if it intersects the ROI
    if (pt1 != pt2)
    {
      cv::line(linesImage, pt1, pt2, cv::Scalar(0,0,0), 2, cv::LINE_AA);

      // Midpoint of the lines
      cv::Point midpoint((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2);
      midpoints.push_back(midpoint); // Add the midpoint to the vector

      // Draw midpoint in red
      cv::circle(linesImage, midpoint, 3, cv::Scalar(0, 0, 255), -1);

      // Print the midpoint
      ROS_INFO("Midpoint of line %zu: (%d, %d)", i, midpoint.x, midpoint.y);
      
    }
}

// Declare a boolean variable 'is_cross' and initialize it to false
bool is_cross = false;

// Get the size of the image in the input message
cv::Size size = cv_ptr->image.size();

// Extract the width and height of the image from the 'size' object
int width = size.width;
int height = size.height;

// Iterate through all the midpoints in the 'midpoints' vector 
for (size_t i = 0; i < midpoints.size(); i++)
{
  for (size_t j = i+1; j < midpoints.size(); j++)
  {
    // Calculate the Euclidean distance between the midpoints 'i' and 'j'
    double distance = cv::norm(midpoints[i] - midpoints[j]);

    // Check if the distance is between 30 and 120 
    if (distance > 30 && distance < 120)
    {
      // Draw a line between the midpoints on the 'linesImage' matrix
      cv::line(linesImage, midpoints[i], midpoints[j], 
      cv::Scalar(0,0,0), 2, cv::LINE_AA);

      // Calculate the midpoint of the line joining the midpoints 'i' and 'j'
      cv::Point midpoint_line((midpoints[i].x + midpoints[j].x) / 2, 
      (midpoints[i].y + midpoints[j].y) / 2);

      // Draw a circle at the midpoint on the 'linesImage' matrix
      cv::circle(linesImage, midpoint_line, 3, cv::Scalar(255, 255, 255), -1);

      // Calculate the distance between the midpoint of the line and 
      // the center of the image
      double distance_to_center = cv::norm(midpoint_line - 
      cv::Point(width/2, height/2));

      // Check if the distance to the center is less than 25 pixels
      if (distance_to_center < 25)
      {
        // If it is, set the 'is_cross' flag to true
        is_cross = true;
      }
    }
  }
}
// cv::imwrite("/home/heiyinwong/comp0129_s23_robot/src/cw3_team_1/image.jpg", linesImage);
// Print the number of valid lines detected
ROS_INFO("Number of valid lines detected: %lu", lines.size());

// Print if cross detected or not
if (is_cross)
{
  ROS_INFO("Cross detected");
  return 1;
}
else
{
  ROS_INFO("Cross not detected");
  return 0;
}

}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief This function performs line detection and orientation estimation on 
 * the input image.
 * @return The estimated orientation in radians.
 */
double 
cw3::detectLinesWithOrientation()
{
  double psi = 0.0;
  cv::Mat src_gray;
  cv::cvtColor(cv_ptr->image, src_gray, cv::COLOR_BGR2GRAY);
  // Apply Canny edge detection
  cv::Mat edges;
  cv::Canny(src_gray, edges, 50, 200, 3);

  // Perform Hough Line Transform
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(edges, lines, 1, CV_PI/180, 106, 0, 0);
  // cv::HoughLines(edges, lines, 1, CV_PI/180, 106, 0, 0);

  // Draw lines on the image
  cv::Mat linesImage = cv_ptr->image.clone();

  // Set the region of interest (ROI) to the entire image
  cv::Rect roi(0, 0, linesImage.cols, linesImage.rows); 

  // Vector to store the midpoints
  std::vector<cv::Point> midpoints;

  for (size_t i = 0; i < lines.size(); i++) 
  {
    // Extract rho and theta values from the current line
    float rho = lines[i][0], theta = lines[i][1];

    // Define two points for the line based on rho and theta
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));

    // Print the points of the line
    ROS_INFO("Line %zu: (%d,%d) - (%d,%d)", i, pt1.x, pt1.y, pt2.x, pt2.y);

    // Clip the line to the ROI
    cv::clipLine(roi, pt1, pt2);

    // Draw the line only if it intersects the ROI
    if (pt1 != pt2)
    {
      cv::line(linesImage, pt1, pt2, cv::Scalar(0,0,0), 2, cv::LINE_AA);

      // Midpoint of the lines
      cv::Point midpoint((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2);
      midpoints.push_back(midpoint); // Add the midpoint to the vector

      // Draw midpoint in red
      cv::circle(linesImage, midpoint, 3, cv::Scalar(0, 0, 255), -1);

      // Print the midpoint
      ROS_INFO("P1 of line %zu: (%d, %d)", i, pt1.x, pt1.y);
      ROS_INFO("P2 of line %zu: (%d, %d)", i, pt2.x, pt2.y);
      double slope = static_cast<double>(pt2.y-pt1.y)/
      static_cast<double>(pt2.x-pt1.x);

      psi = std::atan(slope);      
      if (psi > M_PI) 
      {
        // Subtract 2π if the angle is greater than π
        psi = psi - 2.0 * M_PI; 
      } 
      else if (psi < -M_PI) 
      {
        // Add 2π if the angle is less than -π
        psi = psi + 2.0 * M_PI; 
      } 
      else 
      {
        // The angle is already wrapped to the range of -π to π
        psi = psi; 
      }
      // Print the psi and slope values
      ROS_INFO("psi: %f", psi);
      ROS_INFO("slope: %f", slope);

      // Check if the value of psi is less than or equal to pi/2 and greater 
      // than or equal to -0.01 to check if the rotation is within permissible 
      // range as the order of the lines varies
      if (psi <=M_PI/2 && psi >= -0.01)
      {
        ROS_INFO("Line detection with orientation complete.");
        break;
      }
    }    
  }
  return psi;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Scans for shapes at a given location
 * @param x x-coordinate of the location to scan
 * @param y y-coordinate of the location to scan
 * @param zone the zone of the robot to scan in
 * @return void
 */
void
cw3::scanShape(double x, double y,int zone){

  // Log information about the start of the scan
  ROS_INFO("===> scaning shape at (%f, %f) - zone: %d", x, y, zone);

  // Check if the robot is in zone 14 and adjust the camera offset accordingly
  if (zone ==14)
  {    
    //Zone 8 require a 90 degree rotation on camera to avoid big jump,
    // so here the offset is different
    moveRobotLinearWithRotation(x +CAMERA_OFFSET_Y, y + CAMERA_OFFSET_X, 
    0.6,-M_PI/2);  
  }
  else
  {  
    moveRobotLinear(x +CAMERA_OFFSET_X, y + CAMERA_OFFSET_Y, 0.6);
  }
  
  // By using EuclideanClusterExtraction, we can split the points into cluster
  // Lock the mutex to ensure thread safety
  mutex_pcl_.lock();
  // Create a KdTree object for searching neighbors
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  // Set the input cloud for the KdTree object
  tree->setInputCloud(g_cloud_filtered_);
  // Create a vector to store the indices of the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  // Create an object for extracting Euclidean clusters
  pcl::EuclideanClusterExtraction<PointT> ec;
  // Set the maximum distance between points in a cluster
  ec.setClusterTolerance(0.02); 

  //We have already down sampled the point cloud, 
  //so we set the min cluster size to be 10
  ec.setMinClusterSize(500);  
  // Set the maximum number of points in a cluster
  ec.setMaxClusterSize(10000);
  // Set the search method to be used for finding neighbors
  ec.setSearchMethod(tree);
  // Set the input cloud for the Euclidean cluster extractor
  ec.setInputCloud(g_cloud_filtered_);
  // Extract the clusters and store their indices in the vector
  ec.extract(cluster_indices);
  // Unlock the mutex after the operation is complete
  mutex_pcl_.unlock();

  // Print the number of points in the filtered point cloud
  ROS_INFO("==> number of points in cloud: %ld", 
  g_cloud_filtered_->points.size());

  // Get the number of objects detected by the Euclidean cluster extraction
  int number_of_object = cluster_indices.size();
  ROS_INFO("==> number of object: %d", number_of_object);
  int j = 0;

  // Loop through each cluster and extract the points
  for(const auto& cluster: cluster_indices)
  {
    // Create a new point cloud pointer for the cluster
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

    // Add the points of the cluster to the new point cloud
    for(const auto& index: cluster.indices)
    {
      cloud_cluster->points.push_back(g_cloud_filtered_->points[index]);
    }
    // Set the dimensions of the new point cloud
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Calculate the centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    // Print the centroid coordinates of the object
    ROS_INFO("==> centroid of object %d: (%f, %f, %f)", j, centroid[0], 
    centroid[1], centroid[2]);

    // Transform the centroid point from the camera frame to the world frame
    geometry_msgs::PointStamped centroid_in_camera_frame;
    centroid_in_camera_frame.header.frame_id = FRAME_ID;
    centroid_in_camera_frame.header.stamp = ros::Time(0);
    centroid_in_camera_frame.point.x = centroid[0];
    centroid_in_camera_frame.point.y = centroid[1];
    centroid_in_camera_frame.point.z = centroid[2];
    geometry_msgs::PointStamped centroid_in_world_frame;
    g_listener_.transformPoint(PANDA_FRAME_0, centroid_in_camera_frame, 
    centroid_in_world_frame);

    // Print the centroid coordinates of the object in the world frame
    ROS_INFO("==> centroid of object %d in world frame: (%f, %f, %f)", j, 
    centroid_in_world_frame.point.x, centroid_in_world_frame.point.y, 
    centroid_in_world_frame.point.z);
    
    // Get the current pose of the robot arm and print its coordinates
    geometry_msgs::PoseStamped robot_current_pose = arm_group_.getCurrentPose();

    // Print the distance between the robot arm and the centroid point
    ROS_INFO("======> robot current pose: (%f, %f, %f)", 
    robot_current_pose.pose.position.x-CAMERA_OFFSET_X, 
    robot_current_pose.pose.position.y-CAMERA_OFFSET_Y, 
    robot_current_pose.pose.position.z);    

    // Calculate the distance between the robot arm and the centroid point
    double distance = std::sqrt(std::pow(centroid_in_world_frame.point.x - x, 2)
     + std::pow(centroid_in_world_frame.point.y - y, 2));
     // Print the distance between the robot arm and the centroid point
    ROS_INFO("==> distance between robot and centroid: %f", distance);

    //Check if the shape is already in the list
    bool is_target_in_map = false;
    // Loop through the list of target shapes in the shape position map
    for(TargetShape target: shape_pos_map_)
    {      
      // Check if the difference in x and y coordinates between the target shape 
      // and the current centroid is less than 0.1
      if (std::abs(target.getX() - centroid_in_world_frame.point.x) < 0.1 && 
      std::abs(target.getY() - centroid_in_world_frame.point.y) < 0.1)
      { 
        // If a target shape is found at the current centroid, set the 
        // is_target_in_map flag to true and break out of the loop
        is_target_in_map = true;     
        break;
      }
    }

    // If the current centroid matches a target shape in the shape position map, 
    // log an error message and skip this object
    if (is_target_in_map)
    {
      ROS_ERROR("Target is already in the map, skip this object");
      continue;
    }
    // If the distance between the robot and the centroid is less than 0.25, 
    // move the robot to the centroid using the findShapeAcurateLocation() func.
    // If the distance is greater than or equal to 0.25, log an error message 
    // and skip this object
    if (distance < 0.25)
    { //0.25 before
      //Move to the found centroid
      findShapeAcurateLocation(centroid_in_world_frame,zone);
    }else 
    {
      ROS_ERROR
    ("==> distance between robot and centroid is too large, skip this object");
    }    
    
    j++;
  }

  //Move the robot back to a safe position before moving to the next sector
  if (zone ==14)
  {
    moveRobotLinearWithRotation(x +CAMERA_OFFSET_Y, y + CAMERA_OFFSET_X, 
    0.6,-M_PI/2);
  }
  else
  {
    moveRobotLinear(x +CAMERA_OFFSET_X, y + CAMERA_OFFSET_Y, 0.6);
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Find the accurate location of a shape in the robot's workspace.
 * 
 * @param centroid_in_world_frame The centroid of the shape in the world frame.
 * @param zone The zone in which the shape is located.
 */
void 
cw3::findShapeAcurateLocation(geometry_msgs::PointStamped& 
centroid_in_world_frame,int zone)
{
  ROS_INFO("===> finding shape acurate location at zone %d", zone);
  
  // Compute the distance between the centroid and the robot
  int j = 0;
  is_checking_camera = true;
  double distance = 32767;
  int cluster_size = 0;
  
  //For zone 8, we need to rotate the camera to the left
  //to avoid a big rotation on J1
  double CAM_OFFSET_1 = (zone==14)? CAMERA_OFFSET_Y: -CAMERA_OFFSET_X;
  double CAM_OFFSET_2 = (zone==14)?CAMERA_OFFSET_X : -CAMERA_OFFSET_Y;

  do
  {    
    //Move to the found centroid
    ROS_INFO("--> Moving robot to (camera point): (%f, %f, %f)", 
    centroid_in_world_frame.point.x , centroid_in_world_frame.point.y , 0.6);
    
    if(zone == 14)
    {
      //Need special handling for zone 8 to avoid big rotation on J1
      moveRobotLinearWithRotation(centroid_in_world_frame.point.x + 
      CAM_OFFSET_1, centroid_in_world_frame.point.y + 
      CAM_OFFSET_2, 0.6,-M_PI/2);  
    }
    else
    {
      moveRobotLinear(centroid_in_world_frame.point.x + 
      CAMERA_OFFSET_X, centroid_in_world_frame.point.y + CAMERA_OFFSET_Y, 0.6);
    }
     
    // Perform Euclidean clustering on the point cloud to segment the shape
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    mutex_pcl_.lock();
    tree->setInputCloud(g_cloud_filtered_);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02); 
    //We have already down sampled the point cloud, 
    //so we set the min cluster size to be 10
    ec.setMinClusterSize(500);  
    ec.setMaxClusterSize(10000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(g_cloud_filtered_);
    ec.extract(cluster_indices);
    mutex_pcl_.unlock();

    ROS_INFO("==> number of points in cloud: %ld", 
    g_cloud_filtered_->points.size());
    
    int number_of_object = cluster_indices.size();
    ROS_INFO("==> number of object: %d", number_of_object);
    
    if(cluster_indices.size() > 0)
    {
      //Check the cloest object (to handle multiple objects in the camera view)
      double d = 32767;
      for(const auto& cluster : cluster_indices)
      {
        // Extract the point cloud of the cluster
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for(const auto& index: cluster.indices)
        {
          cloud_cluster->points.push_back(g_cloud_filtered_->points[index]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Calculate the centroid of the cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        ROS_INFO("==> centroid of object %d: (%f, %f, %f)", j, 
        centroid[0], centroid[1], centroid[2]);

        //Transform back to the base frame
        geometry_msgs::PointStamped centroid_in_camera_frame;
        centroid_in_camera_frame.header.frame_id = FRAME_ID;
        centroid_in_camera_frame.header.stamp = ros::Time(0);
        centroid_in_camera_frame.point.x = centroid[0];
        centroid_in_camera_frame.point.y = centroid[1];
        centroid_in_camera_frame.point.z = centroid[2];

        //we only handle the cloest object
        geometry_msgs::PointStamped centroid_in_world_frame_buff; 
        g_listener_.transformPoint(PANDA_FRAME_0, centroid_in_camera_frame, 
        centroid_in_world_frame_buff);
        ROS_INFO("==> centroid of object %d in world frame: (%f, %f, %f)", 
        j, centroid_in_world_frame_buff.point.x, 
        centroid_in_world_frame_buff.point.y, 
        centroid_in_world_frame_buff.point.z);
        
         // Get current pose of robot arm's end-effector
        geometry_msgs::PoseStamped robot_current_pose = 
        arm_group_.getCurrentPose();
        
        // Calculate distance between robot arm's end-effector & object centroid
        double d = std::sqrt(std::pow(centroid_in_world_frame_buff.point.x - 
        robot_current_pose.pose.position.x - CAM_OFFSET_1, 2) + 
        std::pow(centroid_in_world_frame_buff.point.y - 
        robot_current_pose.pose.position.y - CAM_OFFSET_2, 2));

        // Log distance between robot arm's end-effector and object centroid
        ROS_INFO("**==> distance(d) between robot and centroid: %f", d);
        // Log the previously recorded distance
        ROS_INFO("**==> distance(distance) between robot and centroid: %f", 
        distance);

        // Check if the newly calculated distance is smaller than the previously
        // recorded distance
        if (d < distance)
        {
          // If so, update the previously recorded distance with new distance
          distance = d;
           // Update the object's centroid in the world frame with the new value
          centroid_in_world_frame.point.x =centroid_in_world_frame_buff.point.x;
          centroid_in_world_frame.point.y =centroid_in_world_frame_buff.point.y;
          centroid_in_world_frame.point.z =centroid_in_world_frame_buff.point.z;
           // Update the cluster size of the object
          cluster_size = cloud_cluster->points.size();
        }
        j++;
      }
      
      // Get current pose of robot arm's end-effector
      geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
      
       // Log the object's centroid in the world frame
      ROS_INFO("==> centroid of object %d in world frame: (%f, %f, %f)", 
      j, centroid_in_world_frame.point.x, centroid_in_world_frame.point.y, 
      centroid_in_world_frame.point.z);

      // Log the current position of the robot arm's end-effector
      ROS_INFO("==> robot current position: (%f, %f, %f)", 
      current_pose.pose.position.x, current_pose.pose.position.y, 
      current_pose.pose.position.z);

      // Log the current position of the robot arm's end-effector with an offset 
      ROS_INFO("==> robot current position with offset: (%f,%f,%f)", 
      current_pose.pose.position.x - CAM_OFFSET_1, current_pose.pose.position.y 
      - CAM_OFFSET_2, current_pose.pose.position.z);

      // Log the updated object's centroid in the world frame
      ROS_INFO("==> New centroid: (%f, %f, %f)",centroid_in_world_frame.point.x, 
      centroid_in_world_frame.point.y, centroid_in_world_frame.point.z);    

      // Log the distance between the object's centroid in the world frame and 
      // the robot arm's end-effector  
      ROS_INFO("===> Distance between the centroid and the robot: %f",distance);
      
    }

    j++;    

    //Wait for the camera to update
    ros::Duration(0.5).sleep(); 
  }
  // Check if the maximum number of objects to be scanned has been reached or 
  // if the distance between the robot arm's end-effector
  // and the object centroid is smaller than a predefined threshold
  while(distance > 0.02 && j < 30);
  is_checking_camera = false; //to prevent race condition

  std::string shape = classifyShapeCV();
  ROS_WARN("Shape point: (%f, %f, %f)", centroid_in_world_frame.point.x, 
  centroid_in_world_frame.point.y, centroid_in_world_frame.point.z);
  ROS_WARN("Shape: %s", shape.c_str());

  //Check if the shape is already in the list
  bool is_target_in_map = false;
  for(TargetShape target: shape_pos_map_)
  {      
    //check if the difference in x and y positions between the current shape 
    // and the target shape is less than 0.02
    if (std::abs(target.getX() - centroid_in_world_frame.point.x) < 0.02 && 
    std::abs(target.getY() - centroid_in_world_frame.point.y) < 0.02)
    {
      //if the shape is already in the map, set the boolean variable to true 
      // and break the loop
      is_target_in_map = true;     
        break;
      }
  }

  //if the shape is already in the map, print an error message
  if (is_target_in_map)
  {
    ROS_ERROR
    ("Target is already in the map, skip this object (catch at lv 2 scan)");
  }
  //if the shape is not in the map, create a new target object and add it to 
  // the shape_pos_map_ vector
  else
  {
    TargetShape target_object(centroid_in_world_frame.point.x, 
    centroid_in_world_frame.point.y, centroid_in_world_frame.point.z, 
    cluster_size, shape,zone);
    shape_pos_map_.push_back(target_object);
  }
  return ;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Function to find the basket using the camera
This function checks the center pixel of the camera image and determines if it 
is a brown color, which indicates the presence of a basket. It then saves the 
index of the basket location in the basket_idx_ variable.
 */
void
cw3::findBasket(){
  is_checking_camera = true;
  //Wait for the camera to update
  ros::Duration(0.5).sleep(); //Wait for the camera to update
    cv::Mat img = cv_ptr->image.clone();
    
    //Get the center pixel
    int h = img.rows;
    int w = img.cols;
    int center_x = w/2;
    int center_y = h/2;
    cv::Vec3b center_pixel = img.at<cv::Vec3b>(center_y, center_x);

    //print the color  
    // Check if the center pixel is ground  BGR around (126,187,126)
    //Compute MSE with the brown color
    double mse = (std::pow(center_pixel[0] - 58, 2) + std::pow(center_pixel[1] -
     58, 2) + std::pow(center_pixel[2] - 142, 2))/3;

    ROS_WARN("MSE: %f", mse);
    ROS_WARN("Color: (%d, %d, %d)", center_pixel[0], center_pixel[1], 
    center_pixel[2]);

    // Determine basket location based on color
    if (mse < 100)
    {
      ROS_WARN("Found basket on the left side");
      basket_idx_ = 0;      
    }
    else
    {
      ROS_WARN("No basket, so basket is at the other side");
      basket_idx_ = 1;           
    }

    //Draw a circle at the center
    cv::circle(img, cv::Point(center_x, center_y), 10, cv::Scalar(0,0,255), 2);
    // cv::imwrite("/home/heiyinwong/comp0129_s23_robot/src/cw3_team_1/basket.jpg", img);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Classifies the shape in the input image using computer vision 
 * techniques
 * @return A string representing the classified shape: "Cross", "Nought", or 
 * "Unknown"
 */
std::string
cw3::classifyShapeCV()
{
    cv::Mat img = cv_ptr->image.clone();
    
    // Define ROI
    int h = img.rows;
    int w = img.cols;
    int range = 200;
    int roi_x = w/2 - range;
    int roi_y = h/2 - range;
    
    cv::Rect roi(roi_x, roi_y, 2*range, 2*range);

    // Crop image using ROI
    cv::Mat filtered = img(roi);

    // Convert it back to RGB for easier processing
    cv::cvtColor(filtered, filtered, cv::COLOR_BGR2RGB);


    // just like the point cloud, apply filter to each pixel
    filtered.forEach<cv::Vec3b>([](cv::Vec3b &pixel, const int * 
    position) -> void 
    {
        if ((pixel[1] > 40) || (!(((pixel[2]>100) || ((pixel[0]>100) 
        &&(pixel[2]<40))))))
        {
            pixel = cv::Vec3b(0, 0, 0);
        }
    });

    cv::cvtColor(filtered, filtered, cv::COLOR_RGB2GRAY);

    // Convert the image to binary mask; same as python:  img[img>0] = 1
    cv::threshold(filtered, filtered, 0, 255, cv::THRESH_BINARY);

    //Apply mean filter to reduce noise
    cv::blur(filtered, filtered, cv::Size(5, 5));

    //Apply threshold again to make sure the image is binary
    cv::threshold(filtered, filtered, 0, 255, cv::THRESH_BINARY);
  
    // Apply Canny edge detection
    cv::Canny(filtered, filtered, 1, 1, 3);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(filtered, contours, hierarchy, cv::RETR_TREE, 
    cv::CHAIN_APPROX_SIMPLE);

    //Count how many side for the polygon
    int sides = 0;
    ROS_INFO("Number of contours: %ld", contours.size());
    for (int i = 0; i < contours.size(); i++){
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 0.01*cv::arcLength(contours[i], 
        true), true);

        //Tested with python, the valid number should be either 8 or 12 
        if ((approx.size() == 4)||(approx.size() == 8) || (approx.size() == 12) 
        || (approx.size() == 24))
        {
            ROS_INFO("Number of sides: %ld", approx.size());
            sides = approx.size();
            break;
        }
        else
        {          
          ROS_INFO("Rejected: Number of sides: %ld", approx.size());
        }
    }
    ROS_INFO("Number of sides: %d", sides);

    //Draw a circle at the center of the image
    cv::circle(filtered, cv::Point(filtered.cols/2, filtered.rows/2), 
    5, cv::Scalar(255, 255, 255), -1);

    // cv::imwrite("/home/heiyinwong/comp0129_s23_robot/src/cw3_team_1/image.jpg", filtered);

    if ((sides == 12) || (sides ==24))
    {
      ROS_INFO("Shape: Cross");
      return "Cross";
    }
    else if ((sides == 8) || (sides == 4))
    {
      ROS_INFO("Shape: Nought");
      return "Nought";
    }

    ROS_ERROR("Shape: Unknown");
    return "Unknown";
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Move the gripper to the specified width
 * 
 * @param width The target width for the gripper
 *
 * @details This function calculates the joint targets, sets the velocity and
 * acceleration scaling factors, moves the robot hand and returns success or 
 * failure.
 * 
 * @return bool Whether the motion plan was successful or not
 */
bool 
cw3::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > GRIPPER_MAX_OPEN) 
    width = GRIPPER_MAX_OPEN;
  if (width < GRIPPER_CLOSED) 
    width = GRIPPER_CLOSED;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // set the velocity and acceleration scaling factors to increase speed
  hand_group_.setMaxVelocityScalingFactor(1);
  hand_group_.setMaxAccelerationScalingFactor(1);
  
  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

bool
cw3::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Homing the robot to its default position
 *
 * This function moves the robot to its default position, which is defined as
 * a specific geometry_msgs::Pose with a certain orientation. 
 * A Cartesian path is computed and executed to reach the home position.
 * The gripper is also opened to the maximum width during the homing process.
 *
 * @return
 */ 
void
cw3::homingRobot(){
  ROS_INFO("Homing robot");
  
  //Define home position
  geometry_msgs::Pose home_pose;
  home_pose.position.x = 0.5;
  home_pose.position.y = 0.0;
  home_pose.position.z = 0.5;
  home_pose.orientation.x = PICK_ORIENTATION_X;
  home_pose.orientation.y = PICK_ORIENTATION_Y;
  home_pose.orientation.z = PICK_ORIENTATION_Z;
  home_pose.orientation.w = PICK_ORIENTATION_W;

  //Define the path
  std::vector<geometry_msgs::Pose> waypoints = {home_pose};
  moveit_msgs::RobotTrajectory trajectory;
  
  //Compute the Cartesian path, by default, the avoid_collisions flag is true 
  // referring to the doc, so it is not specified here.
  // Return -1.0 in case of error.
  double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, 0.0, 
  trajectory);

  if (fraction < -0.99)
  {    
    ROS_ERROR("Failed to compute cartesian path");
    return;
  }
  //Move the robot
  arm_group_.execute(trajectory);
  ROS_INFO("Moving gripper to max open");

  moveGripper(GRIPPER_MAX_OPEN);

  ROS_INFO("Robot homed successfully");
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Move robot linearly to a given position
 * 
 * This function moves the robot linearly to a given position in the cartesian 
 * space.
 * 
 * @param[in] x The x coordinate of the target position 
 * @param[in] y The y coordinate of the target position
 * @param[in] z The z coordinate of the target position
 * 
 * @return true if the robot reaches the target position, false otherwise
 */
bool
cw3::moveRobotLinear(double x,double y,double z)
{
  // Initialize the pose of the robot
  geometry_msgs::Pose pose = pose;

  // Set the x, y, and z positions of the pose to the desired values
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // Set the orientation of the pose to a fixed value
  pose.orientation.x = PICK_ORIENTATION_X;
  pose.orientation.y = PICK_ORIENTATION_Y;
  pose.orientation.z = PICK_ORIENTATION_Z;
  pose.orientation.w = PICK_ORIENTATION_W;

  // Define a vector of poses that contains only the desired pose
  std::vector<geometry_msgs::Pose> waypoints = {pose};

  // Execute the trajectory defined by the waypoints vector
  bool res = executeRobotTrajectory(waypoints);
  return res;
}

////////////////////////////////////////////////////////////////////////////////
/** Moves the robot in a linear motion while also rotating it to a specified 
 * yaw angle.
 * @param x The x position to move the robot to.
 * @param y The y position to move the robot to.
 * @param z The z position to move the robot to.
 * @param yaw The desired yaw angle to rotate the robot to.
 * @return True if the robot successfully executes the trajectory, else false.
 */
bool
cw3::moveRobotLinearWithRotation(double x,double y,double z, double yaw)
{
  // Define a quaternion to represent the desired orientation of the robot  
  tf2::Quaternion q;

  // Set the roll, pitch, and yaw values of the quaternion
  q.setRPY(M_PI, 0, -M_PI/4-yaw);
  
  // Initialize the pose of the robot
  geometry_msgs::Pose pose = pose;

  // Set the x, y, and z positions of the pose to the desired values
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // Set the orientation of the pose to the desired quaternion value
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  // Define a vector of poses that contains only the desired pose
  std::vector<geometry_msgs::Pose> waypoints = {pose};

  // Execute the trajectory defined by the waypoints vector
  bool res = executeRobotTrajectory(waypoints);
  return res;
}

/**
 * @brief callback function for the depth-registered point cloud topic
 *
 * The function processes the point cloud data received on the depth-registered 
 * point cloud topic and updates the input point cloud, converts it to PCL data 
 * type, and publishes the filtered point cloud.
 *
 * @param[in] cloud_input_msg The message data of the depth-registered point 
 * cloud
 */
////////////////////////////////////////////////////////////////////////////////
void
cw3::depthRegisteredPointCallback(
  const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{    
    // Extract inout point cloud info
    g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;   

    // Convert to PCL data type
    pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc_);
    pcl::fromPCLPointCloud2 (g_pcl_pc_, *g_cloud_ptr_);

    mutex_pcl_.lock();
    applyPT (g_cloud_ptr_, g_cloud_filtered_);
    pubFilteredPCMsg (g_pub_cloud_, *g_cloud_filtered_);
    mutex_pcl_.unlock();
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Applies a series of filters to the input point cloud to remove the 
 * green floor and only keep the red and blue objects.
 * @param in_cloud_ptr input point cloud to be filtered
 * @param out_cloud_ptr output point cloud after filtering
 */
void
cw3::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
//This section performs the following steps:
// Downsize the input point cloud by applying voxel grid downsampling. 
// This reduces the number of points to save computational resources.

// Remove the green floor from the point cloud using conditional removal. 
//It creates a condition based on the packed RGB values of the points and 
//removes those points that meet the condition (in this case, green floor points
// with RGB value less than 40).

// Create a filter that allows only blue or purple objects to pass. It 
//adds a condition based on the packed RGB values of the points, allowing points 
//with blue values greater than a defined threshold (BLUE_THRESHOLD) to pass.

// Further filter the point cloud to retain only red objects and remove 
//the brown basket. It adds a condition based on the packed RGB values of the 
//points, allowing points with red values greater than a defined threshold 
//(RED_THRESHOLD) and blue values less than 40 to pass.

// The ConditionalRemoval filter is set with the input point cloud and the 
//conditions defined in steps 2, 3, and 4.

// Finally, the filter is applied, and the modified point cloud is stored in 
//out_cloud_ptr.
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Executes the robot trajectory defined by waypoints
 * 
 * @param waypoints The waypoints that define the robot trajectory
 * @return true if the robot trajectory was executed successfully, false 
 * otherwise
 */
bool
cw3::executeRobotTrajectory(std::vector<geometry_msgs::Pose> waypoints)
{
  moveit_msgs::RobotTrajectory trajectory;
  
  // Compute the Cartesian path, by default, the avoid_collisions flag is true 
  // referring to the doc, so it is not specified here.
  // Return -1.0 in case of error.  
  // double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, 0.0, 
  // trajectory);
  double fraction = arm_group_.computeCartesianPath(waypoints, 0.01, 0.0,
  trajectory);

  bool result = (fraction > -0.99);
  if (!result)
  {    
    ROS_ERROR("Failed to compute cartesian path");
    return false; 
  }

  //Move the robot
  arm_group_.execute(trajectory);
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Publishes filtered point cloud message
 * 
 * @param pc_pub Publisher for the filtered point cloud data
 * @param pc Filtered point cloud data
 * 
 * @return void
 */
void
cw3::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg_);
  pc_pub.publish (g_cloud_filtered_msg_);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * Move nought to the basket.
 * 
 * @param x The x coordinate of the object.
 * @param y The y coordinate of the object.
 * @param goal_point The point where the object will be dropped off.
 */
void
cw3::moveNoughtToBasket(double x, double y,geometry_msgs::Point goal_point)
{ 
// The code performs the following steps:

// Set the flag to indicate that the camera is being checked.
// Home the robot arm to its initial position.
// Get the current joint state target of the arm group.
// Calculate the angle between the y-axis and the object point.
// Wrap the angle between -pi and pi to ensure it stays within that range.
// Set the target joint angle to move the arm to the object position.
// Move the arm to the target joint angle.
// Move the robot linearly to the object point with an offset for the camera.
// Detect the orientation angle of the object using an image.
// Set a quaternion to deal with the rotation based on the detected orientation 
//angle.
// Set the offset for the gripper position.
// Rotate the offset by the orientation angle.
// Move the robot linearly to the object point with the gripper offset and the 
//orientation angle.
// Open the gripper.
// Move the robot linearly down to the object with the appropriate rotation.
// Close the gripper to grasp the object.
// Move the robot linearly to the goal point.
// Set the joint target to a specific value to prevent collisions.
// Move the robot linearly to specific positions to reach the goal point.
// Open the gripper to drop the object.
// Move the robot linearly back up and return to the object position.
// Home the robot arm to its initial position.
}

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Moves the cross to the basket.
 * @param x The x-coordinate of the cross.
 * @param y The y-coordinate of the cross.
 * @param goal_point The goal point to move the cross to.
 */
void
cw3::moveCrossToBasket(double x, double y,geometry_msgs::Point goal_point)
{ 
// The code performs the following steps:
// Sets a flag to indicate that the camera is being checked.
// Homes the robot, which is a process of moving it to a known starting 
// position.
// Calculates the target joint angles for the robot arm based on the given 
// coordinates.
// Moves the robot to the calculated joint angles.
// Moves the robot linearly to the object point with an offset for the camera.
// Detects the orientation angle of the object using an image.
// Sets up the rotation and position offsets for the gripper.
// Moves the robot linearly to the object point with the gripper offset and 
// orientation angle.
// Opens and closes the gripper.
// Moves the robot to the goal point, dropping the object at the specified 
// location.
// Finally, the robot returns to its home position.
}