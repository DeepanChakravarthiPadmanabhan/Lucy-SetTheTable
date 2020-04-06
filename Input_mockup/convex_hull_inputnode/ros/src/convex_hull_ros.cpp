/* Author: Muhammed Umer Ahmed Khan, Deepan Chakravarthi Padmanabhan, Sathiya Ramesh
 * 
 * This code reads a Point Cloud Data and publishes a convex hull as pointcloud,
 * polygonstamped, bounding box. 
 *
 */
#include <ros/ros.h>
// Library
#include<convex_hull_inputnode/convex_hull_ros.h>
#include<iostream>
#include <pcl_ros/transforms.h>
#include <mas_perception_libs/bounding_box.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <mcr_scene_segmentation/impl/helpers.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>


using mas_perception_libs::BoundingBox;
using mas_perception_libs::Color;

// Creating objects for the point cloud library function
// Publisher, Subscriber definitions
ConvexHullROS::ConvexHullROS():
    nh_("~"),
    event_msg_received_(false),
    coefficients (new pcl::ModelCoefficients),
    cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
    inliers (new pcl::PointIndices),
    cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>),
    bounding_box_visualizer_("table_bounding_box_marker", Color(Color::SEA_BLUE))
    {
    sub_event_in_ = nh_.subscribe("event_in",1,&ConvexHullROS::eventInCallback, this);
    publish_convex_hull_ = nh_.advertise<geometry_msgs::PolygonStamped>("convex_hull_input", 1);
    publish_pc_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pointcloud", 1);
    publish_bb_ = nh_.advertise<mcr_perception_msgs::BoundingBox>("table_bounding_box", 1);
    
    }

// Closing the node after the task is accomplished
ConvexHullROS::~ConvexHullROS()
{
  sub_event_in_.shutdown();
}

// Event message to trigger the convex hull construction after receiving the point cloud
void ConvexHullROS::eventInCallback(const std_msgs::String::ConstPtr &msg)
{
  event_msg_received_ = true;
}

// Convex hull construction using RANSAC library in PCL
void ConvexHullROS::update()
{
  if (event_msg_received_)
  {
  // Input Point cloud data file name from ros-parameter  
  nh_.getParam("input_table_pcd_file",input_table_pcd_file);
  
  reader.read (input_table_pcd_file, *cloud);
  // Set the frame of the received PCD to 'camera' and transform it to 'base' frame
  cloud->header.frame_id = "camera";
  pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
  bool transformed = pcl_ros::transformPointCloud("base", *cloud, *cloud, tf_listener);
  if (!transformed)
  {
      return;
  }
  // Publish point cloud in the topic specified above
  publish_pc_.publish(cloud);

  // Build a filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2, 3.0);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-2.0, 0.6);
  pass.filter (*cloud_filtered);

  // Create the segmentation object
  seg.setOptimizeCoefficients (true);
  
  // Constructing the convex hull plane using RANSAC library
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.005);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  
  // Project the model inliers
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
 
  // Create a convex Hull representation of the projected inliers
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  // Write the convex hull to a file for testing purpose
  writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);
  
  // Expection if the Point cloud has no inlier for convex hull
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  // Fit bounding box for the convex hull
  Eigen::Vector3f normal(1.0, 0.0, 0.0);
  BoundingBox box = BoundingBox::create(cloud_hull->points, normal);
  mcr_perception_msgs::BoundingBox ros_bounding_box;
  convertBoundingBox(box, ros_bounding_box);

  // Publish the bounding box in the topic specified above
  bounding_box_visualizer_.publish(ros_bounding_box, "base");
  publish_bb_.publish(ros_bounding_box);

  // set the frame of the polygon convex hull for testing purpose
  polygon_convexhull_.header.frame_id = "base";
  polygon_convexhull_.header.stamp = ros::Time::now();

  // Creating the polygonstamped msg of the convex hull point cloud 
  for(int i =0; i< cloud_hull->points.size (); i++)
  {
    pclpoint.x= cloud_hull->points[i].x;
    pclpoint.y= cloud_hull->points[i].y;
    pclpoint.z= cloud_hull->points[i].z;
    polygon_convexhull_.polygon.points.push_back(pclpoint);
  }
  // Publish the convex hull as polygonstamped msg for testing purpose
  publish_convex_hull_.publish(polygon_convexhull_);
  
  }
}
