#ifndef ICPSPARSEDEMO_H
#define ICPSPARSEDEMO_H

#include <iostream>
#include "ObjLoader.h"
#include "IcpOptimizer.h"

using namespace std;
using namespace Eigen;

class IcpSparseDemo
{
public:
  IcpSparseDemo()
  {
    
  }
  void run()
  {
    //First test with bunny
    cout << "Running demo for the bunny file" << endl;
    
    //Parameters
    size_t kNormals = 10;       //Number of nearest neighbours in order to estimate the normals
    const int nbIterations = 50;//Number of iterations for the algorithm
    const int nbIterationsIn = 2;//Number of iterations for the step 2 of the algorithm 
    const double mu = 10.;      //Parameter for step 2.1
    const int nbIterShrink = 3; //Number of iterations for shrink step (2.1 also)
    const double p = 0.5;       //We use the norm L_p
    const bool verbose = false; //Verbosity trigger
    const IcpMethod method = pointToPlane; //Underlying ICP method

    //Finding the media directory
    string mediaDir = string(ICPSPARSE_MEDIA_DIR);
    mediaDir = mediaDir.substr(1,mediaDir.length()-2);

    //Loading the point clouds
    ObjectLoader myLoader;
    Matrix<double,Dynamic,3> pointCloudOne = myLoader(mediaDir+"bunny_side1.obj");
    Matrix<double,Dynamic,3> pointCloudTwo = myLoader(mediaDir+"bunny_side2.obj");

    //Creating an IcpOptimizer in order to perform the sparse icp
  IcpOptimizer* myIcpOptimizer = new IcpOptimizer(pointCloudOne,pointCloudTwo,kNormals,nbIterations,nbIterationsIn,mu,nbIterShrink,p,method,verbose);
    
    //Perform ICP
    myIcpOptimizer->performSparceICP();
    PointCloud resultingCloud = myIcpOptimizer->getMovedPointCloud();
    myLoader.dumpToFile(resultingCloud,myIcpOptimizer->getMovedNormals(),mediaDir+"bunny_ICP_test.ply");
    cout << "Resulting point cloud is in media/bunny_ICP_test.ply" << endl;
    delete myIcpOptimizer;
  }
  
private:
};

#endif