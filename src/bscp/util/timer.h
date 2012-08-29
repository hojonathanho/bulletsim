#ifndef _timer_h
#define _timer_h

#include <sys/time.h>

class Timer {
  public: 
    Timer() {
      gettimeofday(&start_tv, NULL); 
    }
    void restart() {
      gettimeofday(&start_tv, NULL);
    }
    void reset() { restart(); } 
    double elapsed() {
      gettimeofday(&tv, NULL); 
      return  (tv.tv_sec - start_tv.tv_sec) +
        (tv.tv_usec - start_tv.tv_usec) / 1000000.0;
    }

  private:
    struct timeval tv;
    struct timeval start_tv;

};

#endif
