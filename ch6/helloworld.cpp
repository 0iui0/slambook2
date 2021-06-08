//
// Created on 2021/6/8.
//
// 12(10−x)2.
#include "ceres/ceres.h"
#include "glog/logging.h"

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.

struct CostFunc {
  template <typename T> bool operator()(const T *const x, T *residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

int main() {
  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;
  // Build the problem.
  ceres::Problem problem;
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  ceres::CostFunction *costFunction =
      new ceres::AutoDiffCostFunction<CostFunc, 1, 1>(new CostFunc);
  problem.AddResidualBlock(costFunction, nullptr, &x);

  // Run the solver!
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  return 0;
}