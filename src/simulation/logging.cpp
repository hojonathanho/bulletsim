#include "logging.h"

#include <boost/thread/mutex.hpp>

#include <log4cplus/logger.h>
#include <log4cplus/consoleappender.h>
#include <log4cplus/layout.h>
using namespace log4cplus;
using namespace log4cplus::helpers;

static boost::mutex loggingInitMutex;
static bool loggingInitialized = false;

void LoggingInit() {
    boost::mutex::scoped_lock lock(loggingInitMutex);

    if (loggingInitialized) {
        return;
    }
    loggingInitialized = true;

    SharedObjectPtr<Appender> append_1(new ConsoleAppender());
    append_1->setName(LOG4CPLUS_TEXT("First"));

    tstring pattern = LOG4CPLUS_TEXT("[%-5p %b:%L] %m%n");
    append_1->setLayout( std::auto_ptr<Layout>(new PatternLayout(pattern)) );
    Logger::getRoot().addAppender(append_1);
}
