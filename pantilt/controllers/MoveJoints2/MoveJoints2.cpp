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

#define TIME_STEP 50
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;


double sinMovement(double initTheta, double finTheta, double initTime, double finTime, double t){
  double pos;
  pos = initTheta + ((finTheta - initTheta)/2)*(1-cos(M_PI*(t - initTime)/ (finTime - initTime)));
  return pos;
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

  while (robot->step(TIME_STEP) != -1) {
    double t = robot->getTime();
    if(t == -1){
        break;
    }else if(t <= 2.0){
      double newPanPos = sinMovement(0.0, -M_PI/4, 0.0, 2.0, t);
      double newTiltPos = sinMovement(0.0, -M_PI/6+0.2, 0.0, 2.0, t);
      panPos.push_back(newPanPos);
      tiltPos.push_back(newTiltPos);
      rotMotors[0]->setPosition(newPanPos);
      rotMotors[1]->setPosition(newTiltPos);
      cout << "time: " << t << ", pan:" << newPanPos << ", tilt: " << newTiltPos << endl;
    }else if (t <= 5){
      double newPanPos = sinMovement(-1 * (M_PI / 4), M_PI / 4, 2.0 + TIME_STEP * 0.001, 5.0, t);
      double newTiltPos = sinMovement(-1 * (M_PI / 6) + 0.2, -1 * (M_PI / 6) + 0.25, 2.0 + TIME_STEP * 0.001, 5.0, t);
      panPos.push_back(newPanPos);
      tiltPos.push_back(newTiltPos);
      rotMotors[0]->setPosition(newPanPos);
      rotMotors[1]->setPosition(newTiltPos);
      cout << "time: " << t << ", pan:" << newPanPos << ", tilt: " << newTiltPos << endl;
    }else if(t <= 7){
      double newPanPos = sinMovement(M_PI / 4, 0.0, 5.0 + TIME_STEP * 0.001, 7.0, t);
      double newTiltPos = sinMovement(-1 * (M_PI / 6) + 0.25, 0.0, 5.0 + TIME_STEP * 0.001, 7.0, t);
      panPos.push_back(newPanPos);
      tiltPos.push_back(newTiltPos);
      rotMotors[0]->setPosition(newPanPos);
      rotMotors[1]->setPosition(newTiltPos);
      cout << "time: " << t << ", pan:" << newPanPos << ", tilt: " << newTiltPos << endl;
    }else{
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
