/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_BOUNDING_BOX_VISUALIZER_H
#define MCR_SCENE_SEGMENTATION_BOUNDING_BOX_VISUALIZER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <mcr_perception_msgs/BoundingBox.h>
#include <mas_perception_libs/color.h>

using mas_perception_libs::Color;

namespace mcr
{

namespace visualization
{

class BoundingBoxVisualizer
{
public:
    BoundingBoxVisualizer(ros::NodeHandle *nh, const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    BoundingBoxVisualizer(const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    void setColor(Color color);

    void setLineWidth(float widthX, float widthY);

    void setLineWidth(float width);

    void publish(const mcr_perception_msgs::BoundingBox& box, const std::string& frame_id, const std::string& namspace_name="bounding_boxes");

    void publish(const std::vector<mcr_perception_msgs::BoundingBox>& boxes, const std::string& frame_id, const std::string& namspace_name="bounding_boxes");

    int getNumSubscribers();

private:
    ros::Publisher marker_publisher_;

    // Update color to be changable, using method setColor
    const Color color_;
    Color color_updated_;
    bool check_subscribers_;
    float width_X_, width_Y_;
};

}  // namespace visualization

}  // namespace mcr
#include "impl/bounding_box_visualizer.hpp"

#endif  // MCR_SCENE_SEGMENTATION_BOUNDING_BOX_VISUALIZER_H
