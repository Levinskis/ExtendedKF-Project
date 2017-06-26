#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

 //reference:https://en.wikipedia.org/wiki/Root-mean-square_deviation

VectorXd rmse(4);
rmse << 0,0,0,0;

  if (estimations.size() != ground_truth.size()){

    std::cerr << "estimations and ground_truth vectors are not equal"<< std::endl;
    std::cerr << "In file:"<<__FILE__<<"Line number:"<<__LINE__<< std::endl;
    return rmse;

  }

  for(int i = 0; i < estimations.size(); i++){
        VectorXd deviation = estimations[i].array() - ground_truth[i].array();
        deviation = deviation.array() * deviation.array();
        rmse = rmse.array() + deviation.array();
    }
    //calculate the mean
    rmse = rmse / estimations.size();
    //calculate the squared root
    rmse = rmse.array().sqrt();


    return rmse;



}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
