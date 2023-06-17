#include <cw3_class.h> // change to your team name here!

int main(int argc, char **argv)
{
  ros::init(argc,argv, "cw3_solution_node");
  ros::NodeHandle nh;

  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // create an instance of the cw3 class
  //The cw3 class must be defined after the spinner, as we are homing the 
  // robot in the constructor
  cw3 cw_class(nh);

  // loop rate in Hz
  ros::Rate rate(10);

  while (ros::ok()) 
  {
    // spin and process all pending callbacks
    ros::spinOnce();

    // sleep to fulfill the loop rate
    rate.sleep();
  }
}