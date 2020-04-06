#ifndef CONVEX_HULL_INPUTNODE_CONVEX_HULL_ROS_H
#define CONVEX_HULL_INPUTNODE_CONVEX_HULL_ROS_H

//Messages
#include <std_msgs/String.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <string>
//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mcr_scene_segmentation/bounding_box_visualizer.h>

using mcr::visualization::BoundingBoxVisualizer;
// Using the mcr_scene_segmentation BoundingBoxVisualizer library
class ConvexHullROS
{
private:
    // Node
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher publish_convex_hull_;
    ros::Publisher publish_pc_;
    ros::Publisher publish_bb_;

    // Subscribers
    ros::Subscriber sub_event_in_;

    // ROS messages
    geometry_msgs::PolygonStamped polygon_convexhull_;
    geometry_msgs::PolygonStamped polygon_convexhull_temp_;
    geometry_msgs::Point32 pclpoint;
    bool event_msg_received_;

    // Function declaration
    void eventInCallback(const std_msgs::String::ConstPtr &msg);

    // PCD filename input string
    std::string input_table_pcd_file;

    // PCL variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cloud_filtered, cloud_projected, cloud_hull;
    pcl::PCDReader reader;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers;
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    pcl::PCDWriter writer;

    // ROS attributes
    tf::TransformListener tf_listener;
    BoundingBoxVisualizer bounding_box_visualizer_;
    

public:
    public:
    // Node
    ConvexHullROS();
    virtual ~ConvexHullROS();

    void update();

};

#endif //CONVEX_HULL_INPUTNODE_CONVEX_HULL_ROS_H
