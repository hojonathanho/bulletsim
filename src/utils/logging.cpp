#include "logging.h"

#include <log4cplus/logger.h>
#include <log4cplus/consoleappender.h>
#include <log4cplus/layout.h>
using namespace log4cplus;
using namespace log4cplus::helpers;

void LoggingInit() {
    SharedObjectPtr<Appender> append_1(new ConsoleAppender());
    append_1->setName(LOG4CPLUS_TEXT("First"));

    //log4cplus::tstring pattern = LOG4CPLUS_TEXT("%d{%m/%d/%y %H:%M:%S,%Q} [%t] %-5p %c{2} %%%x%% - %m [%l]%n");
    tstring pattern = LOG4CPLUS_TEXT("[%-5p %d %b:%L] %m%n");
    append_1->setLayout( std::auto_ptr<Layout>(new PatternLayout(pattern)) );
    Logger::getRoot().addAppender(append_1);
}
