#ifndef _LOGGING_H_
#define _LOGGING_H_

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

#define LOG_TRACE(s) LOG4CPLUS_TRACE(log4cplus::Logger::getRoot(), s)
#define LOG_DEBUG(s) LOG4CPLUS_DEBUG(log4cplus::Logger::getRoot(), s)
#define LOG_INFO(s) LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), s)
#define LOG_WARN(s) LOG4CPLUS_WARN(log4cplus::Logger::getRoot(), s)
#define LOG_ERROR(s) LOG4CPLUS_ERROR(log4cplus::Logger::getRoot(), s)
#define LOG_FATAL(s) LOG4CPLUS_FATAL(log4cplus::Logger::getRoot(), s)

void LoggingInit();

#endif // _LOGGING_H_
