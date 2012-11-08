#include "logging.h"

#include <log4cplus/logger.h>
#include <log4cplus/consoleappender.h>
#include <log4cplus/layout.h>
#include <log4cplus/streams.h>
#include <log4cplus/helpers/loglog.h>
#include <log4cplus/helpers/stringhelper.h>
#include <log4cplus/helpers/property.h>
#include <log4cplus/spi/loggingevent.h>
#include <boost/format.hpp>


using namespace log4cplus;
using namespace log4cplus::helpers;

#define COLOR_NORMAL "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"

class ColorizedConsoleAppender : public ConsoleAppender {
public:
  void append(const spi::InternalLoggingEvent& event)
  {
    char* prefix = NULL;
    char* color = NULL;
    thread::MutexGuard guard (helpers::getLogLog().mutex);

    if (event.getLogLevel() <= DEBUG_LOG_LEVEL) {
      color = COLOR_GREEN;
      prefix = "DEBUG";
    }
    else if (event.getLogLevel() <= INFO_LOG_LEVEL) {
      color = COLOR_NORMAL;
      prefix = "INFO";
    }
    else if (event.getLogLevel() <= WARN_LOG_LEVEL) {
      color = COLOR_YELLOW;
      prefix = "WARN";
    }
    else if (event.getLogLevel() <= ERROR_LOG_LEVEL) {
      color = COLOR_RED;
      prefix = "ERROR";
//      prefix =
    }
    else if (event.getLogLevel() <= FATAL_LOG_LEVEL) {
      color = COLOR_RED;
      prefix = "FATAL";
    }
    tostream& output = (logToStdErr ? tcerr : tcout);
    output << boost::format("%s[%5s] %s%s\n") % color % prefix % event.getMessage().c_str() % COLOR_NORMAL;

      if(immediateFlush) {
          output.flush();
      }
  }
};

void LoggingInit() {
    SharedObjectPtr<Appender> append_1(new ColorizedConsoleAppender());
    append_1->setName(LOG4CPLUS_TEXT("First"));

    //log4cplus::tstring pattern = LOG4CPLUS_TEXT("%d{%m/%d/%y %H:%M:%S,%Q} [%t] %-5p %c{2} %%%x%% - %m [%l]%n");
//    tstring pattern = LOG4CPLUS_TEXT("[%-5p %d %b:%L] %m%n");
    tstring pattern = LOG4CPLUS_TEXT("[%-5p %b:%L] %m%n");
    append_1->setLayout( std::auto_ptr<Layout>(new PatternLayout(pattern)) );
    Logger::getRoot().addAppender(append_1);
}
