#ifndef ICP_OPTIMIZER_H
#define ICP_OPTIMIZER_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "nanoflann.hpp"

class IcpOptimizer
{
public:
  IcpOptimizer();
  std::vector<int> computeCorrespondances(Eigen::Matrix<double,Eigen::Dynamic,3> refCloud, Eigen::Matrix<double,Eigen::Dynamic,3> queryCloud, bool verbose = false);
private:
  std::vector<Eigen::Vector3d> firstCloud;
  std::vector<Eigen::Vector3d> secondCloud;
};

#endif
