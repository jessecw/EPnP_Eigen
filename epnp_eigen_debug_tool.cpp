/* Copyright 2018, Jesse Chen
* This class offers methods for debugging Eigen programs.
*/
#include "epnp_eigen_debug_tool.h"

#include <iostream>
#include <fstream>


const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
const static Eigen::IOFormat MatlabFormat(Eigen::FullPrecision, 0, ", ", "\n", "", "", "[", "]");

void EPnPEigenDebugTool::writeToCSVFile(const string& fileName, Eigen::MatrixXd& mat){
  ofstream csvfile(fileName.c_str());

  csvfile << mat.format(CSVFormat);    
}


void EPnPEigenDebugTool::readFromCSVFile(const string& fileName, Eigen::MatrixXd& mat){
  ifstream indata;
  indata.open(fileName);
  string line;
  unsigned int rows = 0, cols;

  while(getline(indata, line)){
    stringstream lineStream(line);
    string cell;
    cols = 0;
    while (getline(lineStream, cell, ',')){
      mat(rows, cols) = stod(cell);
      cols++;
    }  	
    rows++;
    cout << endl;
  }	

  indata.close();

}


void EPnPEigenDebugTool::printForMatlab(const string& matName, Eigen::MatrixXd& mat){
  cout << matName << mat.format(MatlabFormat);
  cout << ";" << endl;	
}

void EPnPEigenDebugTool::printForMatlab(const string& vecName, Eigen::VectorXd& vec){	
  cout << vecName << vec.format(MatlabFormat);	
  cout << ";" << endl;
}

void EPnPEigenDebugTool::printForMatlab(const string& matName, Eigen::Matrix3d& mat){
  cout << matName << mat.format(MatlabFormat);
  cout << ";" << endl;    
}

void EPnPEigenDebugTool::printForMatlab(const string& vecName, Eigen::Vector3d& vec){
  cout << vecName << vec.format(MatlabFormat);  
  cout << ";" << endl;  
}

