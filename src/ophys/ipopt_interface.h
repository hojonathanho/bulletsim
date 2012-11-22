#pragma once

#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>

#include <Eigen/Dense>
#include <boost/function.hpp>

using namespace Ipopt;
using namespace Eigen;

class BoxConstrainedOptProblem : public TNLP {
public:

  static const double INF = 1e20;

  typedef boost::function<double(const Eigen::Map<const VectorXd> &)> ObjectiveFunc;

  struct Solution {
	SolverReturn status;
    Index n; VectorXd x, z_L, z_U;
    Index m; VectorXd g, lambda;
    double obj_value;
  };

  BoxConstrainedOptProblem(int dim, const VectorXd &lb, const VectorXd &ub, const VectorXd &start, const ObjectiveFunc &objFunc)
    : m_dim(dim), m_lb(lb), m_ub(ub), m_start(start), m_objFunc(objFunc)
  {
    assert(lb.size() == m_dim);
    assert(ub.size() == m_dim);
    assert(start.size() == m_dim);

    assert(sizeof(Number) == sizeof(double));
  }

  virtual ~BoxConstrainedOptProblem() { }

  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = m_dim;
    m = 0;
    nnz_jac_g = 0;
    nnz_h_lag = m_dim*m_dim;
    index_style = C_STYLE;
    return true;
  }

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u) {
    assert(n == m_dim);

    for (int i = 0; i < n; ++i) {
      x_l[i] = m_lb[i];
      x_u[i] = m_ub[i];
    }

    for (int i = 0; i < m; ++i) {
      g_l[i] = -INF;
      g_u[i] = +INF;
    }

    return true;
  }

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda) {
    assert(init_x && !init_z && !init_lambda);
    memcpy(x, m_start.data(), n*sizeof(double));
    return true;
  }

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
    obj_value = m_objFunc(Eigen::Map<const VectorXd>(x, n, 1));
    return true;
  }

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
    // finite differences
    static const double eps = 1e-4;
    VectorXd ex(n);
    memcpy(ex.data(), x, n*sizeof(double));
    Eigen::Map<const VectorXd> wrapper(ex.data(), n, 1);
    for (int i = 0; i < n; ++i) {
      ex[i] = x[i] + eps;
      double b = m_objFunc(wrapper);
      ex[i] = x[i] - eps;
      double a = m_objFunc(wrapper);
      ex[i] = x[i];
      grad_f[i] = (b - a) / (2.*eps);
    }
    return true;
  }

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {
    return false;
  }

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values) {
    return false;
  }

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values) {
    // using bfgs hessian approximations
    return false;
  }

  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
         const IpoptData* ip_data,
         IpoptCalculatedQuantities* ip_cq) {

  	m_soln.status = status;
  	m_soln.n = n; m_soln.x = arrayToEigen(n, x); m_soln.z_L = arrayToEigen(n, z_L); m_soln.z_U = arrayToEigen(n, z_U);
  	m_soln.m = m; m_soln.g = arrayToEigen(m, g); m_soln.lambda = arrayToEigen(m, lambda);
  	m_soln.obj_value = obj_value;
  }

  Solution &solution() { return m_soln; }


protected:
  const int m_dim;
  VectorXd m_lb, m_ub, m_start;
  ObjectiveFunc m_objFunc;
  Solution m_soln;

  static VectorXd arrayToEigen(int n, const double *a) {
  	VectorXd v(n);
  	memcpy(v.data(), a, n*sizeof(double));
  	return v;
  }


private:
  BoxConstrainedOptProblem(const BoxConstrainedOptProblem&);
  BoxConstrainedOptProblem& operator=(const BoxConstrainedOptProblem&);

};
