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
/**
 *  Angle normalization to [-Pi, Pi]
 */
void Tools::AngleNormalization(double *ang) {
    while (*ang > M_PI) *ang -= 2. * M_PI;
    while (*ang < -M_PI) *ang += 2. * M_PI;
}

void Tools::ComputeNIS(MeasurementPackage meas_package, MatrixXd S)
{
    // Measurements
    VectorXd z = meas_package.raw_measurements_;
    // Calculate NIS
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
        NIS_radar_ = z.transpose() * S.inverse() * z;
        cout << NIS_radar_ << "\n ";

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ // Lidar
        NIS_laser_ = z.transpose() * S.inverse() * z;
        cout << NIS_laser_ << "\n ";
    }
}
VectorXd Tools::PolarToCartesian(const VectorXd x)
{
    Eigen::VectorXd cartesian(5);

    double px = x(0) * std::cos(x(1));
    double py = x(0) * std::sin(x(1));
    double vx = x(2) * std::cos(x(1));
    double vy = x(2) * std::sin(x(1));
    double v  = sqrt(vx * vx + vy * vy);

    cartesian << px, py, v, vx, vy;

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
