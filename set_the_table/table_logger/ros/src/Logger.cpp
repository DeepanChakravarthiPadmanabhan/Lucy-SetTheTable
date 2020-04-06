#include "../include/Logger.h"

#include <fstream>

#include "../include/Helper.h"

namespace LogManager{

Logger::Logger()
    : m_pNodeHandle(NULL), m_AllowLocalLogging(true), m_AllowPublishLogging(false), m_LocalFileName(DEFAULTLOGFILENAME) 
{
    SetLocalLogger(this->m_AllowLocalLogging);
    SetPublishLogger(this->m_AllowPublishLogging);
}

Logger* Logger::Instance(){
    if(m_pInstance == NULL)
        m_pInstance = new Logger;
    return m_pInstance;
}

void Logger::SetNodeHandle(ros::NodeHandle* nh)
{
    this->m_pNodeHandle = nh;
    SetPublishLogger(this->m_AllowPublishLogging);
}

void Logger::SetLocalLogger(bool allowLocalLogging, std::string localFileName)
{
    this->m_AllowLocalLogging = allowLocalLogging;
    this->m_LocalFileName = localFileName;    
}

void Logger::SetPublishLogger(bool allowPublishLogging)
{
    this->m_AllowPublishLogging = allowPublishLogging; 
     
    if(this->m_AllowPublishLogging)
        Init();
}

void Logger::LogMessage(std::string messageType, std::string message)
{
    set_the_table::LoggerMessage loggerMesage;

    loggerMesage.Header.stamp = ros::Time::now();
    loggerMesage.Header.seq = m_SequenceNumber++;
    loggerMesage.logType = messageType;
    loggerMesage.message = message;
    loggerMesage.nodeName = ros::this_node::getName();

    LogMessage(loggerMesage);
}

void Logger::LogInfo(std::string message)
{
    LogMessage("Info", message);
}

void Logger::LogError(std::string message)
{
    LogMessage("Error", message);
}

// Private Methods

bool Logger::Init()
{
    if(this->m_pNodeHandle != NULL)
    {
        this->m_Publisher = this->m_pNodeHandle->advertise<set_the_table::LoggerMessage>(PUBLISHTOPICNAME, 20);

        // Added some sleep publisher needs to register with the master. Unless it is regiestered with the master, the messages sent within that time are lost. 
        // In case the master is consuming a lot of load this time might get increased.
        ros::Duration(1).sleep();
    }
    else
        throw "Invalid ROS Node Handle. Cannot proceed with publishing";
}

void Logger::LogMessage(set_the_table::LoggerMessage& message)
{
    if(IsLocalLoggingAllowed())
        WriteMessage(message);
    if(IsPublishLoggingAllowed())
        PublishMessage(message);

}

bool Logger::IsLocalLoggingAllowed()
{
    return m_AllowLocalLogging;
}

bool Logger::IsPublishLoggingAllowed()
{
    return m_AllowPublishLogging;
}

void Logger::PublishMessage(set_the_table::LoggerMessage& message )
{
    this->m_Publisher.publish(message);
}

void Logger::WriteMessage(set_the_table::LoggerMessage& message)
{
    // Reference : http://www.cplusplus.com/reference/fstream/fstream/open/
    std::fstream fs;
    fs.open (this->m_LocalFileName.c_str(), std::fstream::in | std::fstream::out | std::fstream::app);

    fs << ConvertLoggerMessage2String(message).c_str();

    fs.close();
}




} // End of Namespace "LogManager"