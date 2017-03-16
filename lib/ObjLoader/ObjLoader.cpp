#include "ObjLoader.h"

using namespace std;
using namespace Eigen;

ObjectLoader::ObjectLoader()
{
  
}

vector<Eigen::Vector3d> ObjectLoader::operator()(string filePath)
{
  vector<Eigen::Vector3d> pointCloud;
  ifstream filein(filePath.c_str());
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
    pointCloud.push_back(Vector3d(stod(itemsInString[1]),
                                    stod(itemsInString[2]),
                                    stod(itemsInString[3])));
  } 
  cout << "Point Cloud successfully loaded with " << pointCloud.size() << " vertice." << endl;
}
