#pragma once

#ifndef _BOUNDINGBOXVISUALIZER_WRAPPER_H_FILE_
#define _BOUNDINGBOXVISUALIZER_WRAPPER_H_FILE_

#include <mcr_scene_segmentation/bounding_box_visualizer.h>
#include <mas_perception_libs/color.h>
#include <std_msgs/ColorRGBA.h>
#include <string>

using mas_perception_libs::Color;

class BoundingBoxVisualizerWrapper
{

    public:
        BoundingBoxVisualizerWrapper(const std::string& topic_name);
        ~BoundingBoxVisualizerWrapper();
        void publish(const mcr_perception_msgs::BoundingBox& box, const std_msgs::ColorRGBA color, const std::string& frame_id, const std::string& namespace_name="Bounding_Box");
        void publish_wrapper(const std::string& box, const std::string& color, const std::string& frame_id, const std::string& namespace_name);
        void publish_list(std::vector<mcr_perception_msgs::BoundingBox>& boxes, std::vector<std_msgs::ColorRGBA> colors, const std::string& frame_id, const std::string& namespace_name="Bounding_Box");

    private:
        const std::string& topic_name;
        bool check_subscribers_;
        ros::NodeHandle *nh;
        int namespace_counter;
        mcr::visualization::BoundingBoxVisualizer *visualizer;

};



#endif // _BOUNDINGBOXVISUALIZER_WRAPPER_H_FILE_