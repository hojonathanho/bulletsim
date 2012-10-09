#pragma once
#include "utils/config.h"
#include <string>

struct SQPConfig : Config {
  static double collCoefInit;
  static int nStepsInit;
  static double lengthCoef;
  static bool topCollOnly;
  static int plotDecimation;
  static bool pauseEachIter;
  static double distPen;
  static double distDiscSafe;
  static double distContSafe;
  static int maxIter;
  static double trExpand, trShrink, trThresh;
  static double doneIterThresh;

  SQPConfig() : Config() {
    params.push_back(new Parameter<double>("collCoefInit", &collCoefInit, "coeff for collision cost"));
    params.push_back(new Parameter<int>("nStepsInit", &nStepsInit,"initial number of steps in trajectory"));
    params.push_back(new Parameter<double>("lengthCoef", &lengthCoef, "coeff for quadratic length penalty"));
    params.push_back(new Parameter<bool>("topCollOnly", &topCollOnly, "if link has collision, don't bother with its children"));
    params.push_back(new Parameter<int>("plotDecimation", &plotDecimation, "plot every k grippers"));
    params.push_back(new Parameter<bool>("pauseEachIter", &pauseEachIter, "pause each iteration"));
    params.push_back(new Parameter<double>("distPen", &distPen, "distance to start penalizing"));
    params.push_back(new Parameter<double>("distDiscSafe", &distDiscSafe, "safety distance for discrete collision checking"));
    params.push_back(new Parameter<double>("distContSafe", &distContSafe, "safety distance for continuous collision checking"));
    params.push_back(new Parameter<int>("maxIter", &maxIter, "max iterations for sqp procedure"));
    params.push_back(new Parameter<double>("trShrink", &trShrink, "shrink factor"));
    params.push_back(new Parameter<double>("trExpand", &trExpand, "expand factor"));
    params.push_back(new Parameter<double>("trThresh", &trThresh, "threshold to expand/shrink trust region"));
    params.push_back(new Parameter<double>("doneIterThresh", &doneIterThresh, "done iterating threshold"));
  }
};

#ifdef __CDT_PARSER__
    #undef BOOST_FOREACH
    #define BOOST_FOREACH(a, b) for(a; ; )
#endif
