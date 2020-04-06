#include "../include/bounding_box_visualizer_wrapper.h"
#include "../include/helper.h"

#include <boost/python.hpp>

BoundingBoxVisualizerWrapper::BoundingBoxVisualizerWrapper(const std::string& topic_name)
  : topic_name(topic_name)
{
  this->check_subscribers_ = false;
  this->nh = new ros::NodeHandle();
  this->namespace_counter = 0;
  mas_perception_libs::Color default_color(100,100,100);
  this->visualizer = new mcr::visualization::BoundingBoxVisualizer(this->nh, from_python<std::string>(this->topic_name), default_color, this->check_subscribers_);
  this->visualizer->setLineWidth(0.01);
}

BoundingBoxVisualizerWrapper::~BoundingBoxVisualizerWrapper()
{
  delete this->visualizer;
}
void BoundingBoxVisualizerWrapper::publish_wrapper(const std::string& box, const std::string& color, const std::string& frame_id, const std::string& namespace_name)
{
  this->publish(from_python<mcr_perception_msgs::BoundingBox>(box), from_python<std_msgs::ColorRGBA>(color), from_python<std::string>(frame_id), from_python<std::string>(namespace_name) );
}

void BoundingBoxVisualizerWrapper::publish(const mcr_perception_msgs::BoundingBox& box, const std_msgs::ColorRGBA color, const std::string& frame_id, const std::string& namespace_name)
{
  mas_perception_libs::Color c = ConvertStdColor2MasPerceptionColor(color);
  this->visualizer->setColor(c);
  
  visualizer->publish(box, frame_id, namespace_name + ToString<int>(this->namespace_counter++));  
}

void BoundingBoxVisualizerWrapper::publish_list(std::vector<mcr_perception_msgs::BoundingBox>& boxes, std::vector<std_msgs::ColorRGBA> colors, const std::string& frame_id, const std::string& namespace_name)
{
  if(boxes.size() != colors.size())
  {
    //ToDo: Log error and throw exception
    return;
  }

  for(int i=0; i<boxes.size(); i++)
  {
    this->publish(boxes[i], colors[i], frame_id);
  }

}

BOOST_PYTHON_MODULE(BoundingBoxVisualizerWrapper_CPP)
{
  boost::python::class_<BoundingBoxVisualizerWrapper>("BoundingBoxVisualizerWrapper", boost::python::init<std::string>())
    .def("publish", &BoundingBoxVisualizerWrapper::publish_wrapper)
    .def("publish_list", &BoundingBoxVisualizerWrapper::publish_list)
    ;
}
