#pragma once

#ifndef _LOGGER_H_FILE_
#define _LOGGER_H_FILE_

#include <string>

#include <ros/ros.h>

#include <set_the_table/LoggerMessage.h>

namespace LogManager{
    class Logger{
        private:
        // Singleton Class
        Logger();

        void LogMessage(set_the_table::LoggerMessage &message);

        void PublishMessage(set_the_table::LoggerMessage &message);

        void WriteMessage(set_the_table::LoggerMessage &message);

        bool Init();

        bool IsLocalLoggingAllowed();

        bool IsPublishLoggingAllowed();


        static Logger* m_pInstance;
        const static std::string DEFAULTLOGFILENAME, PUBLISHTOPICNAME;
        static int m_SequenceNumber;

        ros::NodeHandle *m_pNodeHandle;
        ros::Publisher m_Publisher;
        bool m_AllowLocalLogging, m_AllowPublishLogging ;
        std::string m_LocalFileName;

        public:
        static Logger* Instance();

        void SetNodeHandle(ros::NodeHandle* nh);

        void SetLocalLogger(bool allowLocalLogging, std::string localFileName = DEFAULTLOGFILENAME);

        void SetPublishLogger(bool allowPublishLogging);

        void LogMessage(std::string messageType, std::string message);

        void LogInfo(std::string message);

        void LogError(std::string message);
    };

    Logger* Logger::m_pInstance = NULL;
    int Logger::m_SequenceNumber = 1;
    const std::string Logger::DEFAULTLOGFILENAME = "Log.txt";
    const std::string Logger::PUBLISHTOPICNAME = "Log_Topic";
}






#endif