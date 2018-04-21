1 namespace for ceres libs 
=======================
1. namespace for ceres libs 

::

  自动微分函数
  using ceres::AutoDiffCostFunction;
  损失函数
  using ceres::CostFunction;
  using ceres::Problem;
  Solver
  using ceres::Solver;
  Solve
  using ceres::Solve;


2. A templated CostFunctor

residual is : 10 - x

::

  // A templated cost functor that implements the residual r = 10 -
  // x. The method operator() is templated so that we can then use an
  // automatic differentiation wrapper around it to generate its
  // derivatives.
  struct CostFunctor {
    template <typename T> bool operator()(const T* const x, T* residual) const {
      residual[0] = T(10.0) - x[0];
      return true;
    }
  };


3. main function 

::

 int main(int argc, char** argv) {
   google::InitGoogleLogging(argv[0]);
 
   // The variable to solve for with its initial value. It will be
   // mutated in place by the solver.
   double x = 0.5;
   const double initial_x = x;
 
   // Build the problem.
   Problem problem;
 
   // Set up the only cost function (also known as residual). This uses
   // auto-differentiation to obtain the derivative (jacobian).
   CostFunction* cost_function =
       new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
   problem.AddResidualBlock(cost_function, NULL, & x);
 
   // Run the solver!
   Solver::Options options;
   options.minimizer_progress_to_stdout = true;
   Solver::Summary summary;
   Solve(options, &problem, &summary);
 
   std::cout << summary.BriefReport() << "\n";
   std::cout << "x : " << initial_x
             << " -> " << x << "\n";
   return 0;
 }
   
4. Summary 

Flow

- build problem

- 构建cost function 

- run the solver
 

