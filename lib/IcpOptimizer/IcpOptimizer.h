#ifndef ICP_OPTIMIZER_H
#define ICP_OPTIMIZER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <float.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "nanoflann.hpp"

/* Each point in a point cloud is loaded as a line vector, 
but every computation is made with the mathematical convention ! (column vectors)
As a consequence, TransMatrix is a column vector */

typedef Eigen::Matrix<double,3,3> RotMatrix;               //A type for the rotation matrix
typedef Eigen::Matrix<double,3,1> TransMatrix;             //A type for the translation matrix
typedef Eigen::Matrix<double,Eigen::Dynamic,3> PointCloud; //A type for the point clouds

class IcpOptimizer
{
public:
  //Constructor
  IcpOptimizer(Eigen::Matrix<double,Eigen::Dynamic,3> _firstCloud, Eigen::Matrix<double,Eigen::Dynamic,3> _secondCloud, size_t _kNormals, int _nbIterations, double _mu, int _nbIterShrink, double _p);
  
  //The algorithm itself
  std::pair<RotMatrix,TransMatrix> performSparceICP();

  //First step : compute correspondances
  std::vector<int> computeCorrespondances(PointCloud refCloud, PointCloud queryCloud, bool verbose = false);

  //Normal estimation
  Eigen::Matrix<double,Eigen::Dynamic,3> estimateNormals(PointCloud pointCloud, const size_t k);

  //Shrink operator
  TransMatrix shrink(TransMatrix h);

  //Getters
  Eigen::Matrix<double,Eigen::Dynamic,3> getFirstNormals() const;
private:
  const PointCloud firstCloud;
  const PointCloud secondCloud;
  /*I don't use the PointCloud name for the normals in order to distinguish them 
  from the vertice*/
  Eigen::Matrix<double,Eigen::Dynamic,3> firstNormals;
  Eigen::Matrix<double,Eigen::Dynamic,3> secondNormals;

  const size_t kNormals;  //k-nn parameter for normal computation
  const int nbIterations; //number of iterations for the algorithm
  const double mu;        //Parameter for ICP step 2.1
  const int nbIterShrink; //Number of iterations for the shrink part (2.1)
  const double p;         //We use the norm L_p
};

#endif
