
/*

POSITION ONLY FOR 3 DOF

jacobian and forward kinematics functions work correctly
DLS inverse function works correctly 

*/

#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
using namespace BLA;

#define SERVO_MIN 100 // pulse count for 0 deg
#define SERVO_MAX 550 // pulse count for 180 deg TUNE

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // default PCA9685 address

// ROBOT DH PARAMETERS (cm)

const float a[] = {0.0f, 14.5f, 14.5f};
const float d[] = {12.5f, 3.0f, 0.0f};
const float alpha[] = {-M_PI/2.0f, 0.0f, M_PI};

//DH angle in rad that corresponds to servo midpoint 
const float q_home_rad[3] = {PI/2.0, -PI/2.0, 0.0f};  
const float servo_dir[3] = {1.0f,1.0f,1.0f};

// CONTROL PARAMETERS
const float lambda = 0.1; // for DLS pseudoinverse //TWEAK
const float dt = 0.02f; //20 ms loop  //TWEAK

const float DELTA = 1e-4f; // difference step for jacobian computation


// JOINT VARIABLES
// initial joint variables
float q[3] = {PI/2.0f, -PI/2.0, 0.0f};

// DH TRANSFORM 

void dhTransform(float theta, float a, float d, float alpha, Matrix<4,4>& H) {
  H = {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
      sin(theta), cos(theta) * cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
      0, sin(alpha), cos(alpha), d,
      0, 0, 0, 1
  };
}

// FORWARD KINEMATICS 

void forward_kinematics(float* q_in, float& x, float& y,float& z){
   
  Matrix<4,4> H = {1,0,0,0,
                      0,1,0,0,
                      0,0,1,0,
                      0,0,0,1};

  Matrix<4,4> Hi;

  for(int i = 0; i < 3; i++){
    dhTransform(q_in[i], a[i],d[i], alpha[i], Hi);
    H = H * Hi;
  }

  x = H(0,3);
  y = H(1,3);
  z = H(2,3);

}

// COMPUTE JACOBIAN

void computeJacobian(float* q_in, Matrix<3,3>& J) {
    float x0, y0, z0;
    forward_kinematics(q_in, x0, y0, z0);          // current end effector position

    for (int i = 0; i < 3; i++) {
        float q_pert[3] = {q_in[0], q_in[1], q_in[2]};
        q_pert[i] += DELTA;         // perturb joint i by a tiny amount
        float xp, yp, zp;
        forward_kinematics(q_pert, xp, yp, zp);    // end effector position after perturbation
        J(0,i) = (xp - x0) / DELTA;  // dx/dqi
        J(1,i) = (yp - y0) / DELTA;  // dy/dqi
        J(2,i) = (zp - z0) / DELTA;  // dz/dqi
    }
}


// CALCULATE DLS PSEUDOINVERSE

Matrix<3,3> DLSinv(Matrix<3,3>& J) {
  Matrix<3,3> Jt = ~J; //transpose
  Matrix<3,3> JJt   = J * Jt;
    // Add damping to diagonal
  JJt(0,0) += lambda * lambda;
  JJt(1,1) += lambda * lambda;
  JJt(2,2) += lambda * lambda;

  return Jt * Inverse(JJt);

}

// Helper to convert degrees to PCA9685 pulse count
int angleToPulse(int angle) {
    return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setServoAngle(uint8_t channel, int angle) {
    pwm.setPWM(channel, 0, angleToPulse(angle));
}

// convert dh angle in radians to servo degree command
int dhRadtoDeg(float dh_rad, int joint) {

  // how far from home in rad
  float delta_rad = dh_rad - q_home_rad[joint];
  // convert to deg
  float delta_deg = RAD_TO_DEG * delta_rad;
  // direction flip -- TEST
  delta_deg = delta_deg * servo_dir[joint];

  // offset from servo midpoint
  int servo_deg = (int)(90.0f + delta_deg);

  // clamp to valid servo range
  return(constrain(servo_deg, 0, 180));

}

// function to write all servos
void writeServos() {
  for (int i = 0; i < 3; i++) {
    int deg = dhRadtoDeg(q[i], i);
    setServoAngle(i, deg);
  }
}


// PRINT MATRIX (FOR DEBUGGING)
template <int rows, int cols>
void printMatrix(BLA::Matrix<rows, cols> mat) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            Serial.print(mat(i, j));
            Serial.print("\t");  // tab-separated columns
        }
        Serial.println();  // newline after each row
    }
    Serial.println();  // blank line after matrix
}




// ================== SETUP =====================================

void setup(){
  pwm.begin();
  pwm.setPWMFreq(50); // servos run at 50Hz
  Serial.begin(115200);

   // Command home position
    setServoAngle(0, 90);
    setServoAngle(1, 90);
    setServoAngle(2, 90);
    delay(2000);

};

// =================== LOOP =====================================

void loop(){



}

