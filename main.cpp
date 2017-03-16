#include <iostream>
#include "ObjLoader.h"
#include "IcpOptimizer.h"

using namespace std;
using namespace Eigen;

int main()
{
  ObjectLoader myLoader;
  Matrix<double,Dynamic,3> pointCloudOne = myLoader("/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/testPC/bunny_side1.obj");
  Matrix<double,Dynamic,3> pointCloudTwo = myLoader("/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/testPC/bunny_side3.obj");

  IcpOptimizer my_icpOptimizer;
  my_icpOptimizer.computeCorrespondances(pointCloudOne,pointCloudTwo);

  return 0;
}
