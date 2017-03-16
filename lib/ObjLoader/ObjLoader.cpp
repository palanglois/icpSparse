#include "ObjLoader.h"

using namespace std;
using namespace Eigen;

ObjectLoader::ObjectLoader()
{
  
}

Matrix<double,Dynamic,3> ObjectLoader::operator()(string filePath)
{
  Eigen::Matrix<double,Eigen::Dynamic,3> pointCloud;
  ifstream filein(filePath.c_str());
  int iterator = 0;
  for (std::string line; std::getline(filein, line); ) 
  { 
    if(line[0] != 'v')
      continue;
    istringstream iss(line);
    vector<string> itemsInString{istream_iterator<string>{iss},
                      istream_iterator<string>{}};
    if(itemsInString.size() != 4)
    { 
      cout << "Problem when parsing vertex in file : " << filePath << endl;
      continue;
    }
     pointCloud.resize(pointCloud.rows()+1,3);
     pointCloud(iterator,0) = stod(itemsInString[1]);
     pointCloud(iterator,1) = stod(itemsInString[2]);
     pointCloud(iterator,2) = stod(itemsInString[3]);
     iterator++;
    /*pointCloud.push_back(Vector3d(stod(itemsInString[1]),
                                    stod(itemsInString[2]),
                                    stod(itemsInString[3])));*/
  } 
  cout << "Point Cloud successfully loaded with " << pointCloud.rows() << " vertice." << endl;
  return pointCloud;
}
