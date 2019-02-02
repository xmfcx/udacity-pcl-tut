#include <ros/ros.h>
#include <ros/spinner.h>
#include "UdacityTut.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pcldatacamp");
  ros::NodeHandle nh;
  UdacityTut udacity_tut(nh);

  ros::AsyncSpinner spinner(5);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
