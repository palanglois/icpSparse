#include <iostream>
#include "ObjLoader.h"
#include "IcpOptimizer.h"
#include "option_parser.h"
#include "icpSparseDemo.h"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
  //Create parsing options
  op::OptionParser opt;
  opt.add_option("-h", "--help", "show option help");
  opt.add_option("-d","--demo", "demo mode (uses integrated media)");
  opt.add_option("-i1","--input_1","Path to first input obj file (REQUIRED)","");
  opt.add_option("-i2","--input_2","Path to second input obj file (REQUIRED)","");
  opt.add_option("-o","--output","Path to the output directory (REQUIRED)","");
  opt.add_option("-n","--name","Name of the output file","output");
  opt.add_option("-k", "--k_normals", "knn parameter for normals computation", "10" );
  opt.add_option("-n1", "--n_iterations_1","Nb of iterations for the algorithm","50");
  opt.add_option("-n2", "--n_iterations_2","Nb of iterations for the algorithm's step 2","2");
  opt.add_option("-mu","--mu","Parameter for step 2.1","10");
  opt.add_option("-ns","--n_iterations_shrink","Number of iterations for shrink step (2.1)","3");
  opt.add_option("-p","--p_norm","Use of norm L_p","0.5");
  opt.add_option("-po","--point_to_point","Use point to point variant");
  opt.add_option("-pl","--point_to_plane","Use point to plane variant");
  opt.add_option("-v","--verbose","Verbosity trigger");

  //Parsing options
  bool correctParsing = opt.parse_options(argc, argv);
  if(!correctParsing)
    return EXIT_FAILURE;

  //Parameters
  const string first_path = opt["-i1"];
  const string second_path = opt["-i2"];
  string output_path = opt["-o"];
  size_t kNormals = op::str2int(opt["-k"]);
  const int nbIterations = op::str2int(opt["-n1"]);
  const int nbIterationsIn = op::str2int(opt["-n2"]);
  const double mu = op::str2double(opt["-mu"]);
  const int nbIterShrink = op::str2int(opt["-ns"]);
  const double p = op::str2double(opt["-p"]);
  const bool verbose = op::str2bool(opt["-v"]);
  const bool demoMode = op::str2bool(opt["-d"]);
  const bool hasHelp  =  op::str2bool(opt["-h"]);

  const bool isPointToPoint = op::str2bool(opt["-po"]);
  const bool isPointToPlane = op::str2bool(opt["-pl"]);

  //Making checks

  if(demoMode)
  {
    IcpSparseDemo demo;
    demo.run();
    return 0;
  }

  if(hasHelp) 
  {
    opt.show_help();
    return 0;
  }

  if(first_path == "")
  {
    cerr << "Please specify the path of the first object file." << endl;
    opt.show_help();
    return EXIT_FAILURE;
  }

  if(second_path == "")
  {
    cerr << "Please specify the path of the second object file." << endl;
    opt.show_help();
    return EXIT_FAILURE;
  }

  if(output_path == "")
  {
    cerr << "Please specify the path of the output directory." << endl;
    opt.show_help();
    return EXIT_FAILURE;
  }

  if(output_path[output_path.size()-1] != '/')
    output_path.append("/");
  output_path.append(opt["-n"] + ".ply"); 

  if(isPointToPlane && isPointToPoint)
  {
    cerr << "Please choose only one ICP method !" << endl;
    opt.show_help();
    return EXIT_FAILURE;
  }

  IcpMethod method = pointToPoint;
  
  if(isPointToPlane)
    method = pointToPlane;
  else if (isPointToPoint)
    method = pointToPoint;
  else
  {
    cerr << "Please choose at least one ICP method (point to point or point to plane)." << endl;
    opt.show_help();
    return EXIT_FAILURE;
  }

  //Loading the point clouds
  ObjectLoader myLoader;
  Matrix<double,Dynamic,3> pointCloudOne = myLoader(first_path);
  Matrix<double,Dynamic,3> pointCloudTwo = myLoader(second_path);

  //Creating an IcpOptimizer in order to perform the sparse icp
  IcpOptimizer myIcpOptimizer(pointCloudOne,pointCloudTwo,kNormals,nbIterations,nbIterationsIn,mu,nbIterShrink,p,method,verbose);

  //Perform ICP
  myIcpOptimizer.performSparceICP();
  PointCloud resultingCloud = myIcpOptimizer.getMovedPointCloud();

  //Save the resulting point cloud
  myLoader.dumpToFile(resultingCloud,myIcpOptimizer.getMovedNormals(),output_path);

  //Show resulting transformation
  RigidTransfo resultingTransfo = myIcpOptimizer.getComputedTransfo();
  cout << "Computed Rotation : " << endl << resultingTransfo.first << endl << "Computed Translation : " << endl << resultingTransfo.second << endl;

  return 0;
}
