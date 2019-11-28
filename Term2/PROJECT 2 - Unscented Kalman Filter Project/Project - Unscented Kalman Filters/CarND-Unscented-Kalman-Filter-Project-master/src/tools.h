#ifndef TOOLS_H_
#define TOOLS_H_
#include "measurement_package.h"
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
    *Function that converts polar coordinates to cartesian coordinates
   */
  VectorXd PolarToCartesian(const VectorXd x);

  /**
    *Function that converts Cartesian coordinates to polar coordinates
   */
  VectorXd CartesianToPolar(const Eigen::VectorXd cartesian);
  /**
   *  Angle normalization to [-Pi, Pi]
   */
  void AngleNormalization(double *ang);

  /**
   * @brief ComputeNIS
   * @param meas_package
   * @param S
   */
  void ComputeNIS(MeasurementPackage meas_package, MatrixXd S);
private:
  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;



};

#endif /* TOOLS_H_ */
