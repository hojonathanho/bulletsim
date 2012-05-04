#include "utils/logging.h"

int main(int argc, char *argv[]) {
    LoggingInit();
    LOG_ERROR("oh no");
    LOG_FATAL("fatal");
    LOG_INFO("info");
    LOG_TRACE("trace");
    return 0;
}
