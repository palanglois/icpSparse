#include <iostream>
#include "ObjLoader.h"
#include "IcpOptimizer.h"

using namespace std;
using namespace Eigen;

int main()
{

  //Parameters
  size_t kNormals = 10;         //Number of nearest neighbours in order to estimate the normals
  const int nbIterations = 15;  //Number of iterations for the algorithm
  const int nbIterationsIn = 2; //Number of iterations for the step 2 of the algorithm 
  const double mu = 10.;        //Parameter for step 2.1
  const int nbIterShrink = 3;   //Number of iterations for shrink step (2.1 also)
  const double p = 0.5;         //We use the norm L_p

  //Loading the point clouds
  ObjectLoader myLoader;
  Matrix<double,Dynamic,3> pointCloudOne = myLoader("/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/testPC/bunny_side1.obj");
  Matrix<double,Dynamic,3> pointCloudTwo = myLoader("/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/testPC/bunny_side3.obj");

  //Creatin an IcpOptimizer in order to perform the sparse icp
  IcpOptimizer myIcpOptimizer(pointCloudOne,pointCloudTwo,kNormals,nbIterations,nbIterationsIn,mu,nbIterShrink,p);

  myIcpOptimizer.performSparceICP();

  //Saves the computed normal for the first point cloud to a .ply file in order to check algorithm validity
  myLoader.dumpToFile(pointCloudOne,myIcpOptimizer.getFirstNormals(),"/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/testPC/bunny_normals_test.ply");

  //Test the rigid transformation
  /*RotMatrix initialRotation;
  initialRotation.setZero();
  for(int i=0;i<3;i++)
    initialRotation(i,i) = 1.;

  TransMatrix initialTranslation;
  initialTranslation.setZero();
  initialTranslation(0,0) = 1.;
  PointCloud notMovedPc = myIcpOptimizer.movePointCloud(pointCloudOne,RigidTransfo(initialRotation,initialTranslation));
  cout << "Difference should be 0 : " << notMovedPc - pointCloudOne << endl;*/

  return 0;
}
