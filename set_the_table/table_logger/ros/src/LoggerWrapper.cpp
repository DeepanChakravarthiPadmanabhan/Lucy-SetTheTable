
#include "../include/LoggerWrapper.h"
#include "../include/CppPythonConvertor.h"

#include <boost/python.hpp>

LoggerWrapper::LoggerWrapper()
{
    logger = LogManager::Logger::Instance();
    logger->SetNodeHandle(new ros::NodeHandle);
}

void LoggerWrapper::SetLocalLogger(const std::string allowLocalLogging, const std::string localFileName)
{
    bool allowLocalLogging_ = from_python<bool>(allowLocalLogging);
    std::string localFileName_ = from_python<std::string>(localFileName);

    logger->SetLocalLogger(allowLocalLogging_, localFileName_);
}

void LoggerWrapper::SetPublishLogger(const std::string allowPublishLogging)
{
    bool allowPublishLogging_ = from_python<bool>(allowPublishLogging);

    logger->SetPublishLogger(allowPublishLogging_);
}

void LoggerWrapper::LogMessage(const std::string messageType, const std::string message)
{
    std::string messageType_ = from_python<std::string>(messageType);
    std::string message_ = from_python<std::string>(message);

    logger->LogMessage(messageType_, message_);
}

void LoggerWrapper::LogInfo(const std::string message)
{
    std::string message_ = from_python<std::string>(message);

    logger->LogInfo(message_);
}

void LoggerWrapper::LogError(const std::string message)
{
    std::string message_ = from_python<std::string>(message);

    logger->LogError(message_);
}

BOOST_PYTHON_MODULE(LoggerWrapper_CPP)
{
  boost::python::class_<LoggerWrapper>("LoggerWrapper", boost::python::init<>())
    .def("LogMessage", &LoggerWrapper::LogMessage)
    .def("LogInfo", &LoggerWrapper::LogInfo)
    .def("LogError", &LoggerWrapper::LogError)
    .def("SetLocalLogger", &LoggerWrapper::SetLocalLogger)
    .def("SetPublishLogger", &LoggerWrapper::SetPublishLogger)
    ;
}
