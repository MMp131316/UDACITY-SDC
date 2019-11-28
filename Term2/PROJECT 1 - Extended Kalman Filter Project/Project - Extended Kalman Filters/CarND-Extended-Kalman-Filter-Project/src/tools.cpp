#include <iostream>
#include "tools.h"

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

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
    return rmse;
}
VectorXd Tools::PolarToCartesian(const VectorXd x)
{
  Eigen::VectorXd cartesian(4);

  double px = x(0) * std::cos(x(1));
  double py = x(0) * std::sin(x(1));
  double vx = x(2) * std::cos(x(1));
  double vy = x(2) * std::sin(x(1));

  cartesian << px, py, vx, vy;

return cartesian;
}
VectorXd Tools::CartesianToPolar(const VectorXd cartesian)
{
  Eigen::VectorXd x_polar(3);
  const double THRESHOLD = 0.000001;

  double px = cartesian(0);
  double py = cartesian(1);

  double rho = std::sqrt(px*px + py*py);
  double phi = std::atan2(py, px);

  if (rho < THRESHOLD ) { rho = THRESHOLD; }

  double v_rho = (rho > THRESHOLD)? (px*cartesian(2) + py*cartesian(3))/rho : THRESHOLD;

  x_polar << rho, phi, v_rho;

return x_polar;
}
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    const double TH1 = 0.0001;
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

    // Deal with the special case problems
    if (fabs(px) < TH1 and fabs(py) < TH1){
        if(px < 0)
        px = TH1;
        py = TH1;
    }
    // Pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    // Check division by zero
    if(fabs(c1) < TH1){
      c1 = TH1;
    }
	//pre-compute a set of terms to avoid repeated calculation
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}
