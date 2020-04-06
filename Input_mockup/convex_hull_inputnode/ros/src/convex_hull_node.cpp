#include <convex_hull_inputnode/convex_hull_ros.h>

int main( int argc, char** argv)
{
  ros::init(argc, argv, "convex_hull_input");
  // Create node
  ros::NodeHandle nh("~");
  ROS_INFO("[convex hull input node] node started");

  int frame_rate = 30; // in Hz
  ConvexHullROS convex_hull_ros_ ;

  ros::Rate loop_rate(frame_rate);

  while (ros::ok())
  {
    convex_hull_ros_.update();
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}