#include <iostream>
#include "tools.h"
#include "Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd mean(4);
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	float e_size = estimations.size();
	//  * the estimation vector size should equal ground truth vector size
	float g_size = ground_truth.size();
	// ... your code here
	if (g_size == e_size){
		//accumulate squared residuals
		VectorXd residual(4);
		VectorXd acc_residual(4);

		for(int i=0; i < estimations.size(); ++i){
			// ... your code here
			residual = (estimations[i] - ground_truth[i]).array().pow(2);
			acc_residual += residual;
		}
		//calculate the mean
		mean = acc_residual * (1./float(e_size));
		//calculate the squared root
		rmse = mean.array().sqrt();
	}
	//return the result
	return rmse;
}

void Tools::CalculateJacobian(const VectorXd& x_state, MatrixXd& Hjac) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	if (pow(px,2) + pow(py,2) <= 0.001){

		Hjac << px / pow((pow(px,2) + pow(py,2)),0.5),py / pow((pow(px,2) + pow(py,2)),0.5),0,0,
			-py / (pow(px,2) + pow(py,2)),px / (pow(px,2) + pow(py,2)),0,0,
			(py * (vx * py - vy * px)) / pow((pow(px,2) + pow(py,2)),1.5),(py * (- vx * py + vy * px)) / pow((pow(px,2) + pow(py,2)),1.5),px / pow((pow(px,2) + pow(py,2)),0.5),py / pow((pow(px,2) + pow(py,2)),0.5);
	}
}
