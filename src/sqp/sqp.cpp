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

const char* getGRBStatusString(int status) {
	assert(status >= 1 && status <= 13);
	return grb_statuses[status];
}


void ConvexPart::addToModel() {
	m_cnts.reserve(m_exprs.size());
	for (int i=0; i < m_exprs.size(); ++i) {
		m_cnts.push_back(m_model->addConstr(m_exprs[i] <= 0, m_cntNames[i].c_str()));
	}
}

void ConvexPart::removeFromModel() {
	BOOST_FOREACH(GRBConstr& cnt, m_cnts) {
		m_model->remove(cnt);
	}
	BOOST_FOREACH(GRBVar& var, m_vars) {
		m_model->remove(var);
	}
}


double OptimizationProblem::getApproxObjective() {
	return m_model->get(GRB_DoubleAttr_ObjVal);
}


vector<ConvexObjectivePtr> OptimizationProblem::convexifyObjectives() {
	vector<ConvexObjectivePtr> out;
	BOOST_FOREACH(CostPtr& cost, m_costs) {
		out.push_back(cost->convexify());
	}
	return out;
}

vector<ConvexConstraintPtr> OptimizationProblem::convexifyConstraints() {
	vector<ConvexConstraintPtr> out;
	BOOST_FOREACH(ConstraintPtr& cnt, m_cnts) {
		out.push_back(cnt->convexify());
	}
	return out;
}

int OptimizationProblem::convexOptimize() {
	m_model->optimize();
	return m_model->get(GRB_IntAttr_Status);
}

double getSum(vector<ConvexObjectivePtr>& objectives) {
	double sum = 0;
	BOOST_FOREACH(const ConvexObjectivePtr& obj, objectives) {
		sum += obj->m_val;		
	}
	return sum;
}

void OptimizationProblem::setupConvexProblem(const vector<ConvexObjectivePtr>& objectives, const vector<ConvexConstraintPtr>& constraints) {
	m_model->update();
	BOOST_FOREACH(const ConvexConstraintPtr& part, constraints)
		part->addToModel();
	GRBQuadExpr objective(0);
	BOOST_FOREACH(const ConvexObjectivePtr& part, objectives) {
		part->addToModel();
		objective += part->m_objective;
	}
	m_model->setObjective(objective);
	m_model->update();
	
}


OptimizationProblem::OptStatus OptimizationProblem::optimize() {

	vector<ConvexObjectivePtr> objectives = convexifyObjectives();
	double objectiveVal = getSum(objectives);
	vector<ConvexConstraintPtr> constraints = convexifyConstraints();
	setupConvexProblem(objectives, constraints);

	for (int iter=0;; ) {
		

		LOG_INFO_FMT("iteration: %i", iter);		
		
		preOptimize();
		int grbStatus = convexOptimize();
		
		if (grbStatus != GRB_OPTIMAL) {
			LOG_ERROR_FMT("bad GRB status: %s", getGRBStatusString(grbStatus));
			return GRB_FAIL;
		}
		storeValues();
		updateValues();
		postOptimize();
		double approxObjectiveVal = getApproxObjective();
		
		
		vector<ConvexObjectivePtr> newObjectives = convexifyObjectives();
		double newObjectiveVal = getSum(newObjectives);

		double trueImprove = objectiveVal - newObjectiveVal;
		double approxImprove = objectiveVal - approxObjectiveVal;
		double improveRatio = trueImprove / approxImprove;

    LOG_INFO_FMT("true improvement: %.2e.  approx improvement: %.2e.  ratio: %.2f", trueImprove, approxImprove, improveRatio);
		

		if (newObjectiveVal > objectiveVal) {
			LOG_INFO("objective got worse! rolling back and shrinking trust region");			
			rollbackValues();
			m_tra->adjustTrustRegion(SQPConfig::trShrink);
			ConvexConstraintPtr cnt = m_tra->convexify();
			cnt->addToModel();
		}
		else {
						
	    if (improveRatio < SQPConfig::trThresh)
	      m_tra->adjustTrustRegion(SQPConfig::trShrink);
	    else 
	      m_tra->adjustTrustRegion(SQPConfig::trExpand);
			
			if (trueImprove < SQPConfig::doneIterThresh && improveRatio > SQPConfig::trThresh) {
				LOG_INFO("cost didn't improve much. stopping iteration");
				return CONVERGED;
				// XXX maybe we should check to see that it doesn't improve several times in a ow					
			}						
			
			
			BOOST_FOREACH(const ConvexConstraintPtr& part, constraints)
				part->removeFromModel();
			BOOST_FOREACH(const ConvexPartPtr& part, objectives)
				part->removeFromModel();
	
			objectives = newObjectives;
			constraints = convexifyConstraints();
			objectiveVal = newObjectiveVal;
			
			setupConvexProblem(objectives, constraints);

			++iter;

		}
				
		if (iter >= SQPConfig::maxIter) {
			LOG_INFO("reached iteration limit");
			return ITERATION_LIMIT;
		}

		if (m_tra->m_shrinkage < SQPConfig::shrinkLimit) {
			LOG_INFO("trust region shrunk too much. stopping");
			return SHRINKAGE_LIMIT;
		}				
		
	}
}