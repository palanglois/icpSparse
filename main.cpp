#include <iostream>
#include "ObjLoader.h"

using namespace std;

int main()
{
  ObjectLoader myLoader;
  vector<Eigen::Vector3d> pointCloudOne = myLoader("/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/testPC/bunny_side1.obj");
  vector<Eigen::Vector3d> pointCloudTwo = myLoader("/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/testPC/bunny_side3.obj");

  return 0;
}
