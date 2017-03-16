#ifndef OBJECT_LOADER_H
#define OBJECT_LOADER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <Eigen/Core>
#include <Eigen/Dense>

class ObjectLoader
{
public:
  ObjectLoader();
  Eigen::Matrix<double,Eigen::Dynamic,3> operator()(std::string filePath);
  void dumpToFile(Eigen::Matrix<double,Eigen::Dynamic,3> vertice, Eigen::Matrix<double,Eigen::Dynamic,3> normals, std::string filePath);
private:
};

#endif
