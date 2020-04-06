#pragma once

#ifndef _LOGGER_WRAPPER_H_FILE_
#define _LOGGER_WRAPPER_H_FILE_

#include "Logger.h"

class LoggerWrapper
{
    private:
    LogManager::Logger* logger;

    public:
    LoggerWrapper();

    // void SetNodeHandle(ros::NodeHandle* nh);

    void SetLocalLogger(const std::string allowLocalLogging, const std::string localFileName = "Log.txt");

    void SetPublishLogger(const std::string allowPublishLogging);

    void LogMessage(const std::string messageType, const std::string message);

    void LogInfo(const std::string message);

    void LogError(const std::string message);

};


#endif //_LOGGER_WRAPPER_H_FILE_