2 含有多个子函数的极小值
==========================
Flow: 

- 1.初始值 

::

  double x1 =  3.0;
  double x2 = -1.0;
  double x3 =  0.0;
  double x4 =  1.0;


- 2.构造损失函数

::

  struct F1 {
    template <typename T> bool operator()(const T* const x1,
                                          const T* const x2,
                                          T* residual) const {
      // f1 = x1 + 10 * x2;
      residual[0] = x1[0] + T(10.0) * x2[0];
      return true;
    }
  };
  
  struct F2 {
    template <typename T> bool operator()(const T* const x3,
                                          const T* const x4,
                                          T* residual) const {
      // f2 = sqrt(5) (x3 - x4)
      residual[0] = T(sqrt(5.0)) * (x3[0] - x4[0]);
      return true;
    }
  };
  
  struct F3 {
    template <typename T> bool operator()(const T* const x2,
                                          const T* const x4,
                                          T* residual) const {
      // f3 = (x2 - 2 x3)^2
      residual[0] = (x2[0] - T(2.0) * x4[0]) * (x2[0] - T(2.0) * x4[0]);
      return true;
    }
  };
  
  struct F4 {
    template <typename T> bool operator()(const T* const x1,
                                          const T* const x4,
                                          T* residual) const {
      // f4 = sqrt(10) (x1 - x4)^2
      residual[0] = T(sqrt(10.0)) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
      return true;
    }
  };
  

- 3.创建问题

::

  Problem problem;
  // Add residual terms to the problem using the using the autodiff
  // wrapper to get the derivatives automatically. The parameters, x1 through
  // x4, are modified in place.
  //ceres::AutoDiffCostFunction<$`typename CostFunctor`, $`int kNumResiduals`, $`int N0`>
  problem.AddResidualBlock(new AutoDiffCostFunction<F1, 1, 1, 1>(new F1),
                           NULL,
                           &x1, &x2);
  
  problem.AddResidualBlock(new AutoDiffCostFunction<F2, 1, 1, 1>(new F2),
                           NULL,
                           &x3, &x4);
  problem.AddResidualBlock(new AutoDiffCostFunction<F3, 1, 1, 1>(new F3),
                           NULL,
                           &x2, &x3);
  problem.AddResidualBlock(new AutoDiffCostFunction<F4, 1, 1, 1>(new F4),
                           NULL,
                           &x1, &x4);

- 4. 问题求解 

::

  Solver::Options options;
  LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer,
                                              &options.minimizer_type))
      << "Invalid minimizer: " << FLAGS_minimizer
      << ", valid options are: trust_region and line_search.";

  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  Solver::Summary summary;

  solve 的输入为:  solver 的option, 以及要求解的问题problem

  Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << "\n";
