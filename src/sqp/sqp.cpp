#include "sqp.h"
#include "config_sqp.h"
#include "utils/logging.h"
#include <boost/format.hpp>

static const char* grb_statuses[] = { "XXX", //0
    "LOADED",//1
    "OPTIMAL",//2
    "INFEASIBLE",//3
    "INF_OR_UNBD",//4
    "UNBOUDNED",//5
    "CUTOFF",//6
    "ITERATION LIMIT",//7
    "NODE_LIMIT",//8
    "TIME_LIMIT",//9
    "SOLUTION_LIMIT",//10
    "INTERRUPTED",//11
    "NUMERIC",//12
    "SUBOPTIMAL" //13
    };

static GRBEnv* grbEnv;
void initializeGRB() {
 grbEnv = new GRBEnv();
 if (GeneralConfig::verbose > 0) grbEnv->set(GRB_IntParam_OutputFlag, 0);
}
GRBEnv* getGRBEnv() {
  return grbEnv;
}

const char* getGRBStatusString(int status) {
  assert(status >= 1 && status <= 13);
  return grb_statuses[status];
}

inline double sum(const vector<double>& x) {
  double out=0;
  for (int i=0; i < x.size(); ++i) out += x[i];
  return out;
}

vector<double> evalApproxObjectives(const vector<ConvexObjectivePtr>& in) {
  vector<double> out(in.size());
  for (int i=0; i < in.size(); ++i) out[i] = in[i]->m_objective.getValue();
  return out;
}

void Optimizer::printObjectiveInfo(const vector<double>& oldExact,
    const vector<double>& newApprox, const vector<double>& newExact) {
  LOG_INFO("cost | exact | approx-improve | exact-improve | ratio");
  for (int i=0; i < m_costs.size(); ++i) {
    double approxImprove = oldExact[i] - newApprox[i];
    double exactImprove = oldExact[i] - newExact[i];
    LOG_INFO_FMT("%.15s | %.3e | %.3e | %.3e | %.3e", m_costs[i]->getName().c_str(),
                 oldExact[i], approxImprove, exactImprove, exactImprove/approxImprove);
  }
}

ConvexPart::ConvexPart() : m_inModel(false) {}

void ConvexPart::addToModel(GRBModel* model) {
  m_model = model;
  m_cnts.reserve(m_exprs.size());
  for (int i = 0; i < m_exprs.size(); ++i) {
    m_cnts.push_back(m_model->addConstr(m_exprs[i] <= 0, m_cntNames[i].c_str()));
  }
  for (int i = 0; i < m_qexprs.size(); ++i) {
    m_qcnts.push_back(m_model->addConstr(m_exprs[i] >= 0, m_qcntNames[i].c_str()));
  }
  m_inModel = true;
}

void ConvexPart::removeFromModel() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts) {
    m_model->remove(cnt);
  }
  BOOST_FOREACH(GRBConstr& qcnt, m_qcnts) {
    m_model->remove(qcnt);
  }
  BOOST_FOREACH(GRBVar& var, m_vars) {
    m_model->remove(var);
  }
  m_inModel = false;
}

ConvexPart::~ConvexPart() {
  assert(!m_inModel);
}

TrustRegion::TrustRegion() : m_shrinkage(1) {}

void TrustRegion::resetTrustRegion() {
  adjustTrustRegion(1./m_shrinkage);
}

Optimizer::Optimizer() {
  m_model = new GRBModel(*getGRBEnv());
}

Optimizer::~Optimizer() {
  delete m_model;
}

double Optimizer::getApproxObjective() {
  return m_model->get(GRB_DoubleAttr_ObjVal);
}

/*
 void Optimizer::updateValues() {
 BOOST_FOREACH(ValVarMap::value_type& valvar, m_val2var) {
 *m_val2var[valvar.first] = m_val2var.second.get(GRB_DoubleAttr_X);
 }
 }
 */
vector<ConvexObjectivePtr> Optimizer::convexifyObjectives() {
  vector<ConvexObjectivePtr> out;
  BOOST_FOREACH(CostPtr& cost, m_costs) {
    out.push_back(cost->convexify(m_model));
  }
  return out;
}

vector<ConvexConstraintPtr> Optimizer::convexifyConstraints() {
  vector<ConvexConstraintPtr> out;
  BOOST_FOREACH(ConstraintPtr& cnt, m_cnts) {
    out.push_back(cnt->convexify(m_model));
  }
  return out;
}

void Optimizer::addCost(CostPtr cost) {
  m_costs.push_back(cost);
}
void Optimizer::addConstraint(ConstraintPtr cnt) {
  m_cnts.push_back(cnt);
}
void Optimizer::setTrustRegion(TrustRegionPtr tra) {
  m_tra = tra;
  assert(m_cnts.size()==0); //trust region must be first constraint
  addConstraint(tra);
}

int Optimizer::convexOptimize() {
  m_model->optimize();
  return m_model->get(GRB_IntAttr_Status);
}

vector<double> Optimizer::evaluateObjectives() {
  vector<double> out(m_costs.size());
  for (int i=0; i < m_costs.size(); ++i) {
    out[i] = m_costs[i]->evaluate();
  }
  return out;
}

void Optimizer::setupConvexProblem(const vector<ConvexObjectivePtr>& objectives, const vector<
    ConvexConstraintPtr>& constraints) {
  m_model->update();
  BOOST_FOREACH(const ConvexConstraintPtr& part, constraints)
    part->addToModel(m_model);
  GRBQuadExpr objective(0);
  BOOST_FOREACH(const ConvexObjectivePtr& part, objectives) {
    part->addToModel(m_model);
    objective += part->m_objective;
  }
  m_model->setObjective(objective);
  m_model->update();
}

void Optimizer::clearConvexProblem(const vector<ConvexObjectivePtr>& objectives, const vector<
    ConvexConstraintPtr>& constraints) {
  BOOST_FOREACH(const ConvexConstraintPtr& part, constraints)
    part->removeFromModel();
  BOOST_FOREACH(const ConvexPartPtr& part, objectives)
    part->removeFromModel();	
}

#include "utils/utils_vector.h"
Optimizer::OptStatus Optimizer::optimize() {

  ////////////
  double trueImprove, approxImprove, improveRatio; // will be used to check for convergence
  vector<double> newObjectiveVals, objectiveVals;
	bool grbFail=false;
  /////////


  for (int iter = 1;; ++iter) {

    LOG_INFO_FMT("iteration: %i", iter);

    ////// convexification /////////
    // slight optimization: don't evaluate objectives
    // if you just did so while checking for improvement
    vector<ConvexObjectivePtr> objectives = convexifyObjectives();
    if (newObjectiveVals.size() == 0) objectiveVals = evaluateObjectives();
    else objectiveVals = newObjectiveVals;
    double objectiveVal = sum(objectiveVals);
    vector<ConvexConstraintPtr> constraints = convexifyConstraints();
    setupConvexProblem(objectives, constraints);
    ///////////////////////////////////

    LOG_INFO("objectiveVals before: " << objectiveVals);
    for (int i=0; i < m_costs.size(); ++i) std::cout << m_costs[i]->getName() << " " << m_costs[i]->evaluate() << std::endl;


    while (true) { // trust region adjustment


      ////// convex optimization /////////////////////
      preOptimize();
      int grbStatus = convexOptimize();
      if (grbStatus != GRB_OPTIMAL) {
        LOG_ERROR_FMT("bad GRB status: %s. problem written to /tmp/sqp_fail.lp", getGRBStatusString(grbStatus));
	      m_model->write("/tmp/sqp_fail.lp");
				grbFail = true;
				break;
      }
      storeValues();
      updateValues();
      postOptimize();
      //////////////////////////////////////////////

      ////////// calculate new objectives ////////////
      vector<double> approxObjectiveVals = evalApproxObjectives(objectives);
      double approxObjectiveVal = sum(approxObjectiveVals);
      newObjectiveVals = evaluateObjectives();
      double newObjectiveVal = sum(newObjectiveVals);
      printObjectiveInfo(objectiveVals, approxObjectiveVals, newObjectiveVals);
      trueImprove = objectiveVal - newObjectiveVal;
      approxImprove = objectiveVal - approxObjectiveVal;
      improveRatio = trueImprove / approxImprove;
      LOG_INFO_FMT("true improvement: %.2e.  approx improvement: %.2e.  ratio: %.2f", trueImprove, approxImprove, improveRatio);
      //////////////////////////////////////////////

      if (approxImprove < 1e-4) {
        LOG_INFO("not much room to improve in this problem.");
        break;
      }

      if (newObjectiveVal > objectiveVal) {
        LOG_INFO("objective got worse! rolling back and shrinking trust region");
        rollbackValues();
        m_tra->adjustTrustRegion(SQPConfig::trShrink);
        // just change trust region constraint, keep everything else the same
        constraints[0]->removeFromModel(); // first constraint = trust region!
        constraints[0] = m_tra->convexify(m_model);
        constraints[0]->addToModel(m_model);
      }
      else {
        if (improveRatio < SQPConfig::trThresh)
          m_tra->adjustTrustRegion(SQPConfig::trShrink);
        else
          m_tra->adjustTrustRegion(SQPConfig::trExpand);
        break;
      }

    }
		
		clearConvexProblem(objectives, constraints);

    //// exit conditions ///
		if (grbFail) {
			return GRB_FAIL;
		}		
    if (iter >= SQPConfig::maxIter) {
      LOG_INFO("reached iteration limit");
      return ITERATION_LIMIT;
    }

    if (m_tra->m_shrinkage < SQPConfig::shrinkLimit) {
      LOG_INFO("trust region shrunk too much. stopping");
      return SHRINKAGE_LIMIT;
    }
    if (trueImprove < SQPConfig::doneIterThresh && improveRatio > SQPConfig::trThresh) {
      LOG_INFO_FMT("cost improvement below convergence threshold (%.3e < %.3e). stopping", SQPConfig::doneIterThresh, SQPConfig::trThresh);
      return CONVERGED; // xxx should probably check that it improved multiple times in a row
    }
    if (approxImprove < 1e-4) {
      LOG_INFO("no room to improve, according to convexification");
      return CONVERGED;
    }

    ///////////////////////

  }
}
