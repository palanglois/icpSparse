#include "IcpOptimizer.h"

using namespace std;
using namespace Eigen;
using namespace nanoflann;

IcpOptimizer::IcpOptimizer()
{
  //Set parameters
}

/* This function computes each closest point in refCloud for each point in queryCloud using the nanoflann kd-tree implementation
*/
vector<int> IcpOptimizer::computeCorrespondances(Matrix<double,Dynamic,3> refCloud, Matrix<double,Dynamic,3> queryCloud, bool verbose)
{
  //Create an adapted kd tree for the point cloud
  typedef KDTreeEigenMatrixAdaptor< Matrix<double,Dynamic,3> > my_kd_tree_t;

  //Create an index
  my_kd_tree_t   mat_index(3, refCloud, 10 /* max leaf */ );
  mat_index.index->buildIndex();

  vector<int> foundIndex;
  for(int i=0;i<queryCloud.rows();i++)
  {
    //Current query point
    Matrix<double,1,3> queryPoint = queryCloud.block(i,0,1,3);

    //Do a knn search
    const size_t num_results = 1; //We want the nearest neighbour
    vector<size_t>   ret_indexes(num_results);
    vector<double> out_dists_sqr(num_results);

    KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );

    mat_index.index->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParams(10));
    foundIndex.push_back(ret_indexes[0]);

    if(verbose)
    {
      cout << queryPoint(0,0) << " " << queryPoint(0,1) << " " << queryPoint(0,2) << " refPoint" << endl;
      cout << refCloud(ret_indexes[0],0) << " " << refCloud(ret_indexes[0],1) << " " << refCloud(ret_indexes[0],2) << " closestPoint" << endl << endl << endl;
    }
  }
  return foundIndex;
}
