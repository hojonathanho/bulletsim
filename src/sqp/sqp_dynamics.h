class DynamicsSolver : public OptimizationProblem {
public:
	virtual void updateValues();
	virtual void storeValues();
	virtual void rollbackValues();
	void preoptimize();
	void postOptimize();
	
	vector<btRigidBody*> m_bodies;
	MatrixXd getPos(btRigidBody*);
	MatrixXd getVel(btRigidBody*);
	MatrixXd getWrench(btRigidBody*);
};

class Gravity : Cost {
	// mgz
	vector<btRigidBody*> m_objs;
	double evaluate();
	ConvexObjectivePtr convexify();
};

class Rigidity : Cost {
	// sum_{contacts} penetration depth
	// should we divide by the number of contacts per rigid body pair?
	// how many contacts get generated?
	
	vector<btRigidBody*> m_objs;
	double evaluate();
	ConvexObjectivePtr convexify();
};

class DynamicsError : Cost {
	vector<btRigidBody*> m_objs;
};