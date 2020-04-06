
#include <mas_perception_libs/color.h>
#include <std_msgs/ColorRGBA.h>

#include <sstream>
#include <string>

#include <boost/python.hpp>
#include <ros/serialization.h>

using mas_perception_libs::Color;

mas_perception_libs::Color ConvertStdColor2MasPerceptionColor(std_msgs::ColorRGBA color)
{
    mas_perception_libs::Color c(static_cast<uint8_t>(color.r), static_cast<uint8_t>(color.g), static_cast<uint8_t>(color.b));
    return c;    
} 

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

template <typename M>
M from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}
