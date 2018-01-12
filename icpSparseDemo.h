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

  /*
  Run the demo
  */
  void run()
  {
    ///////////////    First test with bunny    /////////////////
    cout << "Running demo for the bunny example" << endl;
    
    //Parameters
    size_t kNormals = 10;       //Number of nearest neighbours in order to estimate the normals
    int nbIterations = 25;      //Number of iterations for the algorithm
    int nbIterationsIn = 2;     //Number of iterations for the step 2 of the algorithm 
    double mu = 10.;            //Parameter for step 2.1
    int nbIterShrink = 3;       //Number of iterations for shrink step (2.1 also)
    double p = 0.5;             //We use the norm L_p
    bool verbose = false;       //Verbosity trigger
    IcpMethod method = pointToPlane; //Underlying ICP method

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
    myIcpOptimizer->saveIter(mediaDir+"bunny_ICP_test.txt");
    cout << "Resulting point cloud is in media/bunny_ICP_test.ply" << endl;
    delete myIcpOptimizer;

    ///////////////    Second test with bunny + 1000 noisy points per file    ////////////
    cout << "Running demo for the bunny + 1000 noisy points per files example" << endl;
    
    //Parameters
    kNormals = 10;         //Number of nearest neighbours in order to estimate the normals
    nbIterations = 25;     //Number of iterations for the algorithm
    nbIterationsIn = 2;    //Number of iterations for the step 2 of the algorithm 
    mu = 10.;              //Parameter for step 2.1
    nbIterShrink = 3;      //Number of iterations for shrink step (2.1 also)
    p = 0.5;               //We use the norm L_p
    verbose = false;       //Verbosity trigger
    method = pointToPlane; //Underlying ICP method
    
    //Loading the point clouds
    pointCloudOne = myLoader(mediaDir+"bunny_noised1.obj");
    pointCloudTwo = myLoader(mediaDir+"bunny_noised2.obj");
   
    //Creating an IcpOptimizer in order to perform the sparse icp
    IcpOptimizer* myIcpOptimizer2 = new IcpOptimizer(pointCloudOne,pointCloudTwo,kNormals,nbIterations,nbIterationsIn,mu,nbIterShrink,p,method,verbose);
    
    //Perform ICP
    myIcpOptimizer2->performSparceICP();
    resultingCloud = myIcpOptimizer2->getMovedPointCloud();
    myLoader.dumpToFile(resultingCloud,myIcpOptimizer2->getMovedNormals(),mediaDir+"bunny_noised_ICP.ply");
    myIcpOptimizer2->saveIter(mediaDir+"bunny_noised_ICP.txt");
    cout << "Resulting point cloud is in media/bunny_noised_ICP.ply" << endl;
    delete myIcpOptimizer2;

  }
  
private:
};

#endif
