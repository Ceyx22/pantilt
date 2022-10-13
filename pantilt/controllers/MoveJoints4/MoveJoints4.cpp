// File:          MoveJoints.cpp
// Date:
// Description:   
// Author:        Fernando Matias
// Modifications:
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#define TIME_STEP 50
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

double sinMovement(double initTheta, double finTheta, double initTime, double finTime, double t){
  double pos;
  pos = initTheta + ((finTheta - initTheta)/2)*(1-cos(M_PI*(t - initTime)/ (finTime - initTime)));
  return pos;
}

Eigen::Matrix<double, 4, 1>cubicSpline(double initTheta, double finTheta, double initVelocity, double finVelocity, double initTime, double finTime){
  //p0 = p(t0 ) = a + bt0 + ct20 + dt30
  // v0 = v(t0 ) = b + 2ct0 + 3dt20
  // pf = p(tf ) = a + btf + ct2f + dt3f
  // vf = v(tf ) = b + 2ctf + 3dt2f
  // Creates a 4x4 matrix for the system of equation
  Eigen::Matrix4d matrix{
  {1.0, initTime, pow(initTime, 2.0), pow(initTime, 3.0)}, 
  {0.0, 1.0, 2.0 * initTime, 3.0 * pow(initTime, 3.0)},
  {1.0, finTime, pow(finTime, 2.0), pow(finTime, 3.0)},
  {0.0, 1.0, 2.0 * finTime, 3.0 * pow(finTime, 2.0)}
  };

  Eigen::Matrix<double, 4, 1> constVec{
    {initTheta, initVelocity, finTheta, finVelocity}
  }; 

  matrix = matrix.inverse();

  Eigen::Matrix<double, 4, 1> solvedMatrix = {matrix * constVec};
  return solvedMatrix; 
}

double calcAngle (Eigen::Matrix<double, 4, 1> matrix, double time){
  //θ(t) = a + bt + ct2 + dt3
  return matrix(0) + (matrix(1) * time) + (matrix(2)*(pow(time, 2.0))) + (matrix(3)*(pow(time, 3)));
}
double calcVel(Eigen::Matrix<double, 4, 1> matrix, double time){
  //θ̇(t) = b + 2ct + 3dt2
  return matrix(1) + (2 * matrix(2) * time )+ (3.0 * matrix(3) * (pow(time, 2.0)));
}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Camera *cam = robot->getCamera("camera");
  
  //initialize camera
  cam->enable(TIME_STEP);
  
  //initialize motors
  Motor *rotMotors[2];
  char motors_Names [2][10] = {"pan", "tilt"};
  
  for (int i = 0; i < 2; i++){
    rotMotors[i] = robot->getMotor(motors_Names[i]);
    //rotMotors[i]->setPosition(0.0);
    //rotMotors[i]->setVelocity(0.0);
  }

  vector<double> panPos;
  vector<double> tiltPos;
  
  panPos.push_back(0.0);
  tiltPos.push_back(0.0);

  //Pan movement spline calculations
  Eigen::Matrix<double, 4, 1> applePanMovement{cubicSpline(0.0, 7.0 * M_PI / 4.0, 0.0, 0.0, 0.0, 2.0)};
  Eigen::Matrix<double, 4, 1> ballPanMovement{cubicSpline(7.0 * M_PI / 4.0, M_PI / 4.0, 0.0, M_PI/2.0, 2.0 ,5.0)};
  Eigen::Matrix<double, 4, 1> origionPanMovement{cubicSpline(M_PI / 4.0, 0.0, M_PI/2.0, 0.0, 5.0, 7.0)};

  //Tilt movement Spline Calculations
  Eigen::Matrix<double, 4, 1> appleTiltMovement{cubicSpline(0.0, M_PI / 6.0 + 0.2, 0.0, 0.0, M_PI / 2.0, 2.0)};
  Eigen::Matrix<double, 4, 1> ballTiltMovement{cubicSpline(M_PI / 6.0 + 0.2, M_PI / 6.0 + 0.15, M_PI/2, 0.0, 2.0, 5.0)};
  Eigen::Matrix<double, 4, 1> origionTiltMovement{cubicSpline(M_PI / 6.0 + 0.15, 0.0, 0.0, 0.0, 5.0, 7.0)};



  while (robot->step(TIME_STEP) != -1) {
    double t = robot->getTime();
    if(t <= 2.0){
      double newPanAngle = abs(calcAngle(applePanMovement, t));
      double newPanVel= abs(calcVel(applePanMovement, t));
      double newTiltAngle = -abs(calcAngle(appleTiltMovement, t));
      double newTiltVel= abs(calcVel(appleTiltMovement, t));

      panPos.push_back(newPanAngle);
      tiltPos.push_back(newTiltAngle);
      rotMotors[0]->setPosition(newPanAngle);
      rotMotors[0]->setVelocity(newPanVel);
      rotMotors[1]->setPosition(newTiltAngle);
      rotMotors[1]->setVelocity(newTiltVel);
      cout << "time: " << t << ", pan angle:" << newPanAngle << ", pan Vel:" << newPanVel << ", tilt angle:" << newTiltAngle << ", tilt Vel:" << newTiltVel << endl;
    }else if (t <= 5){
      double newPanAngle = abs(calcAngle(ballPanMovement, t));
      double newPanVel= abs(calcVel(ballPanMovement, t));
      double newTiltAngle = -abs(calcAngle(ballTiltMovement, t));
      double newTiltVel= abs(calcVel(ballTiltMovement, t));

      panPos.push_back(newPanAngle);
      tiltPos.push_back(newTiltAngle);
      rotMotors[0]->setPosition(newPanAngle);
      rotMotors[0]->setVelocity(newPanVel);
      rotMotors[1]->setPosition(newTiltAngle);
      rotMotors[1]->setVelocity(newTiltVel);
      cout << "time: " << t << ", pan angle:" << newPanAngle << ", pan Vel:" << newPanVel << ", tilt angle:" << newTiltAngle << ", tilt Vel:" << newTiltVel << endl;
    }else if(t <= 7){
      double newPanAngle = abs(calcAngle(origionPanMovement, t));
      double newPanVel= abs(calcVel(origionPanMovement, t));
      double newTiltAngle = -abs(calcAngle(origionTiltMovement, t));
      double newTiltVel= abs(calcVel(origionTiltMovement, t));

      panPos.push_back(newPanAngle);
      tiltPos.push_back(newTiltAngle);
      rotMotors[0]->setPosition(newPanAngle);
      rotMotors[0]->setVelocity(newPanVel);
      rotMotors[1]->setPosition(newTiltAngle);
      rotMotors[1]->setVelocity(newTiltVel);
      cout << "time: " << t << ", pan angle:" << newPanAngle << ", pan Vel:" << newPanVel << ", tilt angle:" << newTiltAngle << ", tilt Vel:" << newTiltVel << endl;
    }else if (t > 7){
        break;
    }
  };
  cout << "panPos = [ ";
  
  for (double i = 0.0; i < panPos.size(); i++) {
        cout << panPos.at(i) << ", ";
  }
  
  cout << "]" << endl << "tiltPos = [";
  for (double i = 0.0; i < tiltPos.size(); i++) {
        cout << tiltPos.at(i) << ", ";
  }
  cout << "]" << endl;
  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
