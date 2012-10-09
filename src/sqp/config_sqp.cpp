#include "config_sqp.h"
double SQPConfig::collCoefInit = 10;
int SQPConfig::nStepsInit = 10;
double SQPConfig::lengthCoef = .5;
bool SQPConfig::topCollOnly = false;
int SQPConfig::plotDecimation = 10;
bool SQPConfig::pauseEachIter = false;
double SQPConfig::distPen = .02;
double SQPConfig::distDiscSafe = .018;
double SQPConfig::distContSafe = .01;
int SQPConfig::maxIter=50;
double SQPConfig::trShrink = .5;
double SQPConfig::trExpand = 1.1;
double SQPConfig::trThresh = .2;
double SQPConfig::doneIterThresh = .01;
