#include <iostream>
#include "tools.h"

#define NUM_ELEMENTS_FOR_RMSE 4

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  VectorXd rmse(NUM_ELEMENTS_FOR_RMSE);
  rmse.fill(0.0);
  
  long len_estimation = estimations.size();
  long len_ground_truth =ground_truth.size();
  
  if(len_estimation==0 || len_ground_truth==0 || len_estimation!=len_estimation){
	cout<<"Invalid Inputs given, please check the length of Inputs."<<endl;
	return rmse;
  }
  
  VectorXd diff(NUM_ELEMENTS_FOR_RMSE);
  for(unsigned long i=0;i<len_estimation;i++){
	diff = estimations[i]-ground_truth[i];
	diff = diff.array()*diff.array();
	rmse += diff;
  }
  
  rmse = rmse/len_estimation;
  rmse = rmse.array().sqrt();
  
  return rmse;

}