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
  static double shapeExpansion;
  static double maxSteps;
  static double maxCollCoef;
  static int maxIter;
  static double trExpand, trShrink, trThresh;
  static double shrinkLimit;
  static double doneIterThresh;
  static bool enablePlot;
  static int padMult;

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
    params.push_back(new Parameter<double>("shapeExpansion", &shapeExpansion, "expand shapes"));
    params.push_back(new Parameter<double>("maxCollCoef", &maxCollCoef, "max collision coefficient"));
    params.push_back(new Parameter<double>("maxSteps", &maxSteps, "maximum number of steps in subdivided traj"));
    params.push_back(new Parameter<int>("maxIter", &maxIter, "max iterations for sqp procedure"));
    params.push_back(new Parameter<double>("trShrink", &trShrink, "shrink factor"));
    params.push_back(new Parameter<double>("trExpand", &trExpand, "expand factor"));
    params.push_back(new Parameter<double>("trThresh", &trThresh, "threshold to expand/shrink trust region"));
    params.push_back(new Parameter<double>("shrinkLimit", &shrinkLimit, "maximum trust region shrinkage"));
    params.push_back(new Parameter<double>("doneIterThresh", &doneIterThresh, "done iterating threshold"));
    params.push_back(new Parameter<bool>("enablePlot", &enablePlot, "enablePlot"));
    params.push_back(new Parameter<int>("padMult", &padMult, "1 if obstacles aren't padded, 2 if they are."));
  }
};

