#include <iostream>
#include <math.h>
#include "FusionEKF.h"


using namespace std;

FusionEKF fusionEKF;
MeasurementPackage meas_package;

static void kalman_init();
static double get_filtered_x();
static double get_filtered_y();
static void set_measurment(double x,double y, long long timestamp);

int main()
{

    //set_measurment()


    //Call ProcessMeasurment(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);


    //get_filtered_xy

}

static void set_measurment(double x,double y, long long timestamp){

  meas_package.sensor_type_ = MeasurementPackage::LASER;
  meas_package.raw_measurements_ = VectorXd(2);
  double px = x;
  double py = y;
  meas_package.raw_measurements_ << px, py;
  meas_package.timestamp_ = timestamp;

}


static double get_filtered_x(){

  double p_x = fusionEKF.ekf_.x_(0);
  return p_x;

}
static double get_filtered_y(){


  double p_y = fusionEKF.ekf_.x_(1);
  return p_y;


}
