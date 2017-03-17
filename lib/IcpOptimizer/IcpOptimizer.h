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
typedef std::pair<RotMatrix,TransMatrix> RigidTransfo;     //A type for the rigid transform
typedef Eigen::Matrix<double,Eigen::Dynamic,3> PointCloud; //A type for the point clouds

class IcpOptimizer
{
public:
  //Constructor
  IcpOptimizer(Eigen::Matrix<double,Eigen::Dynamic,3> _firstCloud, Eigen::Matrix<double,Eigen::Dynamic,3> _secondCloud, size_t _kNormals, int _nbIterations, int _nbIterationsIn, double _mu, int _nbIterShrink, double _p);
  
  //The algorithm itself
  RigidTransfo performSparceICP();

  //First step : compute correspondances
  PointCloud computeCorrespondances(PointCloud refCloud, PointCloud queryCloud, bool verbose = false);

  //Apply rigid transformation to a point cloud
  PointCloud movePointCloud(PointCloud poinCloud, RigidTransfo transfo);

  //Normal estimation
  Eigen::Matrix<double,Eigen::Dynamic,3> estimateNormals(PointCloud pointCloud, const size_t k);

  //Classical rigid transform estimation (point-to-point)
  RigidTransfo rigidTransformEstimation(PointCloud a, PointCloud b);

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

  Eigen::Matrix<double,Eigen::Dynamic,3> lambda; //Lagrange multiplier for step 2.1

  const size_t kNormals;    //K-nn parameter for normal computation
  const int nbIterations;   //Number of iterations for the algorithm
  const int nbIterationsIn; //Number of iterations for the step 2 of the algorithm
  const double mu;          //Parameter for ICP step 2.1
  const int nbIterShrink;   //Number of iterations for the shrink part (2.1)
  const double p;           //We use the norm L_p
};

#endif
