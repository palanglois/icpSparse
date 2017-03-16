#ifndef ICP_OPTIMIZER_H
#define ICP_OPTIMIZER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <float.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "nanoflann.hpp"

class IcpOptimizer
{
public:
  IcpOptimizer(Eigen::Matrix<double,Eigen::Dynamic,3> _firstCloud, Eigen::Matrix<double,Eigen::Dynamic,3> _secondCloud, size_t _kNormals);
  std::vector<int> computeCorrespondances(Eigen::Matrix<double,Eigen::Dynamic,3> refCloud, Eigen::Matrix<double,Eigen::Dynamic,3> queryCloud, bool verbose = false);
  Eigen::Matrix<double,Eigen::Dynamic,3> estimateNormals(Eigen::Matrix<double,Eigen::Dynamic,3> pointCloud, const size_t k);
  Eigen::Matrix<double,Eigen::Dynamic,3> getFirstNormals() const;
private:
  const Eigen::Matrix<double,Eigen::Dynamic,3> firstCloud;
  const Eigen::Matrix<double,Eigen::Dynamic,3> secondCloud;
  Eigen::Matrix<double,Eigen::Dynamic,3> firstNormals;
  Eigen::Matrix<double,Eigen::Dynamic,3> secondNormals;

  size_t kNormals;
};

#endif
