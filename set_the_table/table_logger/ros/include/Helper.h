#pragma once

#ifndef _HELPER_LOGGER_H
#define _HELPER_LOGGER_H

#include <iostream>
#include <fstream>


#include <set_the_table/LoggerMessage.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <string>
#include <sstream>

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

std::string GetSeperator()
{
    return "\t";
}

std::string ConvertLoggerMessage2String(set_the_table::LoggerMessage message)
{
    std::string st_message;
    ros::Duration diff = message.Header.stamp - ros::Time(0);
    // boost::posix_time thistime;
    // thistime = from_time_t(diff);
    // boost::posix_time time_ = boost::posix_time::from_time_t(diff);
    // st_message += std::string to_simple_string(time_);;
    // printf("%s", message.Header.stamp);
    // printf("%ld,%ld\n",message.Header.stamp.sec, message.Header.stamp.nsec);
    st_message += ToString<ros::Time>(message.Header.stamp);
    st_message += GetSeperator();
    st_message += message.nodeName.c_str();
    st_message += GetSeperator();
    st_message += ToString<int>(message.Header.seq);
    st_message += GetSeperator();
    st_message += message.logType.c_str();
    st_message += GetSeperator();
    st_message += message.message.c_str();
    st_message += "\n";
    st_message += "\0";

    return st_message;
}




#endif