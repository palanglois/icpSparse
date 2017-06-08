#include "ObjLoader.h"

using namespace std;
using namespace Eigen;

ObjectLoader::ObjectLoader()
{
  
}

/*This function loads the vertice contained in the 3d .obj file located in filePath
*/
Matrix<double,Dynamic,3> ObjectLoader::operator()(string filePath)
{
  //Counting number of vertice in file
  ifstream fileCount(filePath.c_str());
  int nbVertice = 0;
  for (std::string line; std::getline(fileCount, line); ) 
  { 
    if(line[0] == 'v' && line[1] == ' ')
      nbVertice++;
  }
  fileCount.close();
  
  //Filling the point cloud
  Eigen::Matrix<double,Eigen::Dynamic,3> pointCloud;
  pointCloud.resize(nbVertice,3);
  ifstream filein(filePath.c_str());
  int iterator = 0;
  for (std::string line; std::getline(filein, line); ) 
  { 
    if(line[0] != 'v' || line[1] != ' ')
      continue;
    istringstream iss(line);
    vector<string> itemsInString{istream_iterator<string>{iss},
                      istream_iterator<string>{}};
    if(itemsInString.size() != 4)
    { 
      cout << "Problem when parsing vertex in file : " << filePath << endl;
      continue;
    }
    pointCloud(iterator,0) = stod(itemsInString[1]);
    pointCloud(iterator,1) = stod(itemsInString[2]);
    pointCloud(iterator,2) = stod(itemsInString[3]);
    iterator++;
  }
  cout << "Point Cloud loaded with " << pointCloud.rows() << " vertice." << endl;
  filein.close();
  return pointCloud;
}

/* This functions write a ply file at filePath that contains the vertice and normals as specified in the input variables
*/
void ObjectLoader::dumpToFile(Matrix<double,Dynamic,3> vertice, Matrix<double,Dynamic,3> normals, string filePath)
{
  ofstream fileout(filePath.c_str());
  
  //Writing header
  fileout << "ply" << endl
    << "format ascii 1.0" << endl
    << "element vertex " << vertice.rows() << endl
    << "property float x" << endl
    << "property float y" << endl
    << "property float z" << endl
    << "property float nx" << endl
    << "property float ny" << endl
    << "property float nz" << endl
    << "end_header" << endl;

  //Writing the vertice and normals
  for(int i=0;i<vertice.rows();i++)
    fileout << vertice(i,0) << " " << vertice(i,1) << " " << vertice(i,2) << " " << normals(i,0) << " " << normals(i,1) << " " << normals(i,2) << endl;

}
