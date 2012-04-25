#include "logging.h"

#include <log4cplus/configurator.h>

void LoggingInit() {
    log4cplus::BasicConfigurator().configure();
}
