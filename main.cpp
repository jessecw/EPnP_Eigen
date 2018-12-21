/* main.cpp
*  Author: Jesse Chen
*  Date: 2019-10-02
*/
#include <iostream>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "epnp_eigen_debug_tool.h"
#include "epnp_eigen.h"


using namespace std;


void randperm(int num, double max_range, Eigen::MatrixXd& reference_3d_points) {
  vector<int> temp;

  srand((unsigned)time(NULL));

  for (int i = 0; i < 3*num; i++){
    temp.push_back(rand());  	
  }


  random_shuffle(temp.begin(), temp.end());

  for (int i = 0; i < num; i++){
    //cout << "index = " << i << ": " << temp[3*i]/300.0 << ", " << temp[3*i+1]/300.0 << ", " << temp[3*i+2]/300.0 << endl;
    reference_3d_points(i, 0) = temp[3*i]*max_range/RAND_MAX;
    reference_3d_points(i, 1) = temp[3*i + 1]*max_range/RAND_MAX;
    reference_3d_points(i, 2) = temp[3*i + 2]*max_range/RAND_MAX; 
  }
}


int main(int argc, char* argv[])
{
  EPnPEigenDebugTool eigen_debug_tool;
  Eigen::MatrixXd Pw(100, 3);
  Eigen::MatrixXd pc(100, 2);
  double R_pnp[3][3] = { 0.0 };
  double T_pnp[3] = { 0.0 }; 

  Eigen::Matrix3d K;

  K << 120.0, 0, 321.0,
       0, 122.0, 238.0,
       0,     0,     1;

  //randperm(100, 1000.0, Pw);
  //eigen_debug_tool.printForMatlab("Pw = ", Pw);
  //eigen_debug_tool.writeToCSVFile("reference_3d_points.csv", Pw);

  eigen_debug_tool.readFromCSVFile("reference_3d_points.csv", Pw);
  eigen_debug_tool.readFromCSVFile("reference_2d_points.csv", pc);
  //eigen_debug_tool.printForMatlab("Pc = ", pc);

  EPnPEigen pnpEigenSolver(Pw, pc, K);

  cout << "I: solve pose by EPnPEigen." << endl;

  pnpEigenSolver.computePose();

  return 0;
}

