#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

const float almost_zero = 0.00001;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

 //reference:https://en.wikipedia.org/wiki/Root-mean-square_deviation

  //accumulate RMSE
  VectorXd rmse_acc(4);
  rmse_acc = VectorXd::Zero(4);

  if (estimations.size() == 0)
  {

      std:cerr <<"Estimation vector size is 0!"<< std::endl;

  }
  if (estimations.size() != ground_truth.size()){

    std::cerr << "estimations and ground_truth vectors are not equal"<< std::endl;
    std::cerr << "In file:"<<__FILE__<<"Line number:"<<__LINE__<< std::endl;
    return rmse_acc;

  }


  for(int i = 0; i < estimations.size(); i++){
      VectorXd deviation = estimations[i] - ground_truth[i];
      deviation = deviation.array().pow(2).matrix();
      rmse_acc = rmse_acc + deviation;
    }
    //calculate the mean
    rmse_acc = rmse_acc /  estimations.size();
    //calculate the squared root
    rmse_acc = rmse_acc.array().sqrt().matrix();
    return  rmse_acc;



}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj = MatrixXd::Zero(3,4);


    //State variables
    float px = x_state(0);
   	float py = x_state(1);
   	float vx = x_state(2);
   	float vy = x_state(3);

    if (fabs(px) < almost_zero && fabs(py) < almost_zero){
       px = almost_zero;
       py = almost_zero;
   }

    //Coeficient calculations
    float coef_1 = pow(px, 2) + pow(py, 2);
    float coef_2 = sqrt(coef_1);
    float coef_3 = sqrt(pow(coef_3, 3));

     if (fabs(coef_1) < 0.000001) {

       std::cerr << "Cannot calculate Jacobian, division by zero!"<< std::endl;
       std::cerr << "In file:"<<__FILE__<<"Line number:"<<__LINE__<< std::endl;

       return Hj;

     } else {

         Hj << px / coef_2, py / coef_2, 0, 0,
              -py / coef_1, px / coef_1, 0, 0,
               py * (vx * py - vy * px) / coef_3,
               px * (vy * px - vx * py) / coef_3,
               px / coef_2, py / coef_2;
     }

     return Hj;
}
