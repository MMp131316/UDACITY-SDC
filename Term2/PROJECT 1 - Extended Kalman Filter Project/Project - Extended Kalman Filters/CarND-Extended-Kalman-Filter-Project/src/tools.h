#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
   *Function that converts polar coordinates to cartesian coordinates
  */ 
 VectorXd PolarToCartesian(const VectorXd x);

 /**
   *Function that converts Cartesian coordinates to polar coordinates
  */ 
 VectorXd CartesianToPolar(const Eigen::VectorXd cartesian);



};

#endif /* TOOLS_H_ */
