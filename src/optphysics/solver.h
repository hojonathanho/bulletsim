#ifndef __OPTPHYSICS_SOLVER_H__
#define __OPTPHYSICS_SOLVER_H__

namespace ophys {


class PhysicsSolverImpl;
class PhysicsSolver {
public:

  PhysicsSolver();
  ~PhysicsSolver();

  void setHorizon(int);
  void addObject();
  void updateModel();



private:
  boost::shared_ptr<PhysicsSolverImpl> impl;
};

  
} // namespace ophys

#endif // __OPTPHYSICS_SOLVER_H__
