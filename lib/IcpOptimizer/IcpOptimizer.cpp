#include "IcpOptimizer.h"

using namespace std;
using namespace Eigen;
using namespace nanoflann;

IcpOptimizer::IcpOptimizer(Matrix<double,Dynamic,3> _firstCloud, Matrix<double,Dynamic,3> _secondCloud, size_t _kNormals, int _nbIterations, int _nbIterationsIn, double _mu, int _nbIterShrink, double _p) : 
firstCloud(_firstCloud), secondCloud(_secondCloud), kNormals(_kNormals), nbIterations(_nbIterations), nbIterationsIn(_nbIterationsIn), mu(_mu), nbIterShrink(_nbIterShrink), p(_p)
{
  //Normal estimation
  cout << "Estimating normals for first cloud" << endl;
  firstNormals = estimateNormals(_firstCloud,kNormals);
  cout << "Estimating normals for second cloud" << endl;
  secondNormals = estimateNormals(_secondCloud,kNormals);
  cout << "Done with normal estimation" << endl;

  //Initializing the Lagrange multipliers to 0 for step 2.1
  lambda.resize(firstCloud.rows(),3);
  lambda.setZero();
}

/*
This function is the main implementation of the algorithm where every step are made explicit.
*/
RigidTransfo IcpOptimizer::performSparceICP()
{
  //Initialize the rigid transformation

  RotMatrix initialRotation;
  initialRotation.setZero();
  for(int i=0;i<3;i++)
    initialRotation(i,i) = 1.;

  TransMatrix initialTranslation;
  initialTranslation.setZero();

  //Initialize the point cloud that is going to move
  PointCloud movingPC = firstCloud;

  //Beginning of the algorithm itself
  for(int iter = 0; iter<nbIterations ; iter++)
  {
    cout << "Iteration " << iter << endl;
    //1st step : Computing correspondances
    PointCloud matchPC = computeCorrespondances(secondCloud,movingPC); //y in the paper

    //2nd step : Computing transformation
    RigidTransfo iterTransfo;
    for(int iterTwo = 0; iterTwo<nbIterationsIn;iterTwo++)
    {
      // step 2.1 Computing z
    
      //Compute h
      Matrix<double,Dynamic,3> h = movingPC-matchPC+lambda/mu;
      //Optimizing z with the shrink operator
      Matrix<double,Dynamic,3> z;
      z.resize(h.rows(),3);
      for(int i=0;i<h.rows();i++)
        z.row(i) = shrink(h.row(i).transpose());

      // step 2.2 point-to-point ICP

      //Compute C
      PointCloud c = matchPC + z - lambda/mu;
      //Make a point-to-point ICP iteration
      iterTransfo = rigidTransformEstimation(movingPC,c);

      // step 2.3 Updating the Lagrange multipliers
      PointCloud delta = movePointCloud(movingPC,iterTransfo)-matchPC - z;
      lambda = lambda + mu * delta;
    }

    //3rd step : Updating the moving pointCloud
    movingPC = movePointCloud(movingPC,iterTransfo);
  }

  return pair<RotMatrix,TransMatrix>(initialRotation,initialTranslation);
}

/* 
This function computes each closest point in refCloud for each point in queryCloud using the nanoflann kd-tree implementation. It retunes the point cloud of the closest points of queryCloud.
*/
PointCloud IcpOptimizer::computeCorrespondances(Matrix<double,Dynamic,3> refCloud, Matrix<double,Dynamic,3> queryCloud, bool verbose)
{
  //Create an adapted kd tree for the point cloud
  typedef KDTreeEigenMatrixAdaptor< Matrix<double,Dynamic,3> > my_kd_tree_t;

  //Create an index
  my_kd_tree_t   mat_index(3, refCloud, 10 /* max leaf */ );
  mat_index.index->buildIndex();

  PointCloud nearestPoints;
  nearestPoints.resize(queryCloud.rows(),3);
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

    mat_index.index->findNeighbors(resultSet, &queryPoint[0], SearchParams(10));

    //Updating the resulting point cloud
    nearestPoints(i,0) = refCloud(ret_indexes[0],0);
    nearestPoints(i,1) = refCloud(ret_indexes[0],1);
    nearestPoints(i,2) = refCloud(ret_indexes[0],2);

    if(verbose)
    {
      cout << queryPoint(0,0) << " " << queryPoint(0,1) << " " << queryPoint(0,2) << " refPoint" << endl;
      cout << refCloud(ret_indexes[0],0) << " " << refCloud(ret_indexes[0],1) << " " << refCloud(ret_indexes[0],2) << " closestPoint" << endl << endl << endl;
    }
  }
  return nearestPoints;
}

/*
Move the pointCloud according to the rigid transformation in transfo 
*/
PointCloud IcpOptimizer::movePointCloud(PointCloud pointCloud, RigidTransfo transfo)
{
  RotMatrix rot = transfo.first;
  TransMatrix t = transfo.second;
  return (rot*pointCloud.transpose()+t.replicate(1,pointCloud.rows())).transpose();
}

/* 
This function estimates the normals for the point cloud pointCloud. It makes use of the k nearest neighbour algorithm implemented in FLANN
*/
Matrix<double,Dynamic,3> IcpOptimizer::estimateNormals(Matrix<double,Dynamic,3> pointCloud, const size_t k)
{
  //Create an adapted kd tree for the point cloud
  typedef KDTreeEigenMatrixAdaptor< Matrix<double,Dynamic,3> > my_kd_tree_t;

  //Create an index
  my_kd_tree_t   mat_index(3, pointCloud, 10 /* max leaf */ );
  mat_index.index->buildIndex();

  Matrix<double,Dynamic,3> normals;
  normals.resize(pointCloud.rows(),3);
  for(int i=0;i<pointCloud.rows();i++)
  {
    //Current point for which the normal is being computed
    Matrix<double,1,3> currentPoint = pointCloud.block(i,0,1,3);
    //cout << "Current point : " << currentPoint << endl;
    
    //Do a knn search
    vector<size_t> ret_indexes(k);
    vector<double> out_dists_sqr(k);

    KNNResultSet<double> resultSet(k);
    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

    mat_index.index->findNeighbors(resultSet, &currentPoint[0], SearchParams(10));
    
    //Compute the covariance matrix

    //Compute the barycentre of the nearest neighbours
    Matrix<double,1,3> barycentre;
    for(int j=0;j<3;j++)
    {
      double curVal = 0.;
      for(int neighbour=0;neighbour<k;neighbour++)
      {
        curVal += pointCloud(ret_indexes[neighbour],j);
      }
      barycentre(0,j) = curVal / double(k);
    }
    
    //Compute the centered nearest neighbour matrix
    Matrix<double,Dynamic,3> centeredNN;
    centeredNN.resize(k,3);
    //cout << "[" ;
    for(int j=0;j<k;j++)
    {
      centeredNN(j,0) = pointCloud(ret_indexes[j],0) - barycentre(0,0);
      centeredNN(j,1) = pointCloud(ret_indexes[j],1) - barycentre(0,1);
      centeredNN(j,2) = pointCloud(ret_indexes[j],2) - barycentre(0,2);
    }

    //Compute the covariance matrix
    Matrix<double,3,3> covariance = centeredNN.transpose()*centeredNN;

    //Computing its eigen values
    EigenSolver<Matrix<double,Dynamic,Dynamic> > eigensolver(covariance);

    //Find the indice of the lowest eigen value
    int bestIndice = -1;
    double bestVal = DBL_MAX;
    for(int j=0;j<3;j++)
      if(eigensolver.eigenvalues()(j,0).real()<bestVal)
      {
        bestVal = eigensolver.eigenvalues()(j,0).real();
        bestIndice = j;
      }

    //Filling the normal
    Matrix<double,1,3> normal = eigensolver.eigenvectors().block(0,bestIndice,3,1).normalized().transpose().real();
    normals(i,0) = normal(0,0);
    normals(i,1) = normal(0,1);
    normals(i,2) = normal(0,2);
  }
  return normals;
}

/*
This function is the standard point to plane ICP
a : moving cloud
b : reference cloud
*/
RigidTransfo IcpOptimizer::rigidTransformEstimation(PointCloud a, PointCloud b)
{
  //Centering the point clouds
  Matrix<double,1,3> centerA = a.colwise().sum()/a.rows();
  Matrix<double,1,3> centerB = b.colwise().sum()/b.rows();
  PointCloud aCenter = a - centerA.replicate(a.rows(),1);
  PointCloud bCenter = b - centerB.replicate(b.rows(),1);

  //Computing the cross product matrix W
  Matrix<double,3,3> W;
  W.setZero();
  for(int i=0;i<a.rows();i++)
    W = W + a.row(i).transpose()*b.row(i);

  //Computing singular value decomposition
  JacobiSVD<Matrix<double,3,3>> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

  //Computing the rigid transformation
  RotMatrix rotation = svd.matrixV()*svd.matrixU().transpose();
  TransMatrix translation = (centerB.transpose()-rotation*centerA.transpose()).transpose();
  return RigidTransfo(rotation,translation);
}

/*
This function implements the shrink operator which optimizes the function
f(z) = ||z||_2^p + mu/2*||z-h||_2^2
*/
TransMatrix IcpOptimizer::shrink(TransMatrix h)
{
  double alpha_a = pow((2./mu)*(1.-p),1./(2.-p));
  double hTilde = alpha_a+(p/mu)*pow(alpha_a,p-1);
  double hNorm = h.norm();
  if(hNorm <= hTilde)
    return 0*h;
  double beta = ((alpha_a)/hNorm+1.)/2.;
  for(int i=0;i<nbIterShrink;i++)
    beta = 1 - (p/mu)*pow(hNorm,p-2.)*pow(beta,p-1);
  return beta*h;
}

/* 
Just a getter to the normals of the first cloud (moving cloud)
*/
Matrix<double,Dynamic,3> IcpOptimizer::getFirstNormals() const
{
  return firstNormals;
}
