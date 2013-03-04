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

#define LOG_TRACE_FMT(s, ...) LOG4CPLUS_TRACE_FMT(log4cplus::Logger::getRoot(), s, __VA_ARGS__)
#define LOG_DEBUG_FMT(s, ...) LOG4CPLUS_DEBUG_FMT(log4cplus::Logger::getRoot(), s, __VA_ARGS__)
#define LOG_INFO_FMT(s, ...) LOG4CPLUS_INFO_FMT(log4cplus::Logger::getRoot(), s, __VA_ARGS__)
#define LOG_WARN_FMT(s, ...) LOG4CPLUS_WARN_FMT(log4cplus::Logger::getRoot(), s, __VA_ARGS__)
#define LOG_ERROR_FMT(s, ...) LOG4CPLUS_ERROR_FMT(log4cplus::Logger::getRoot(), s, __VA_ARGS__)
#define LOG_FATAL_FMT(s, ...) LOG4CPLUS_FATAL_FMT(log4cplus::Logger::getRoot(), s, __VA_ARGS__)


void LoggingInit();

#endif // _LOGGING_H_
