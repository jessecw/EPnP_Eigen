/* Copyright 2018, Jesse Chen
* This class offers methods for debugging Eigen programs.
*/
#ifndef EPNP_EIGEN_DEBUG_TOOL_H_
#define EPNP_EIGEN_DEBUG_TOOL_H_

#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;

class EPnPEigenDebugTool {
 public:
  EPnPEigenDebugTool(){}
  ~EPnPEigenDebugTool(){} 	

  void writeToCSVFile(const string& fileName, Eigen::MatrixXd& mat);
  void readFromCSVFile(const string& fileName, Eigen::MatrixXd& mat);
  void printForMatlab(const string& matName, Eigen::MatrixXd& mat);
  void printForMatlab(const string& vecName, Eigen::VectorXd& vec);
  void printForMatlab(const string& matName, Eigen::Matrix3d& mat);
  void printForMatlab(const string& vecName, Eigen::Vector3d& vec);
 private:

}; 


#endif // #define EPNP_EIGEN_DEBUG_TOOL_H_