#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
using namespace BLA;


// =============== MACROS and GLOBALS ===============================================================================================================================

#define SERVO_MIN 100 // pulse count for 0 deg
#define SERVO_MAX 550 // pulse count for 180 deg

// starting position joint angles
#define J0_START M_PI/2
#define J1_START -3*M_PI/4
#define J2_START M_PI/3

// starting ee position coords
#define START_X -3.0
#define START_Y -6.5
#define START_Z 36.75

// link lengths
#define L1 12.5 //cm
#define L2 14.5
#define L3 14.5

#define KP 50.0 // controller gain
#define LAMBDA 0.01 // for DLS pseudoinv
#define DT 0.02 // dt

const float DELTA = 1e-4f; // difference step for jacobian computation

#define CONV_THRESHOLD 1.0 // convergence threshold (cm)
#define MAX_ITER 500

// trajectory parameters
#define END_X 10
#define END_Y 0
#define END_Z 38

#define T_total 3.0f

#define QDOT_MAX 3 // max joint veloc.

// ============== ROBOT ARM CLASS =============================================================================================================

class RobotArm {

  public: 

    static const int DOF = 3;
    float q[DOF];

    float Kp = KP;
    float lambda = LAMBDA;
    float dt = DT;

    //DH angle in rad that corresponds to servo midpoint -- basically offsets
    const float q_home_rad[3] = {PI/2.0, -PI/2.0, 0.0f};  
    const float servo_dir[3] = {1.0f,1.0f,1.0f};

    bool target_reached = false;
    const float pos_threshold = CONV_THRESHOLD;

    Adafruit_PWMServoDriver ServoDriver;

    RobotArm(Adafruit_PWMServoDriver _ServoDriver) {
        q[0] = J0_START;
        q[1] = J1_START;
        q[2] = J2_START;
        ServoDriver = _ServoDriver;
    }

    // member functions 

    // start servo driver
    void init_servo_driver();
    // convert degrees to pulse count 
    int angleToPulse(int angle);
    // set single servo angle 
    void setServoAngle(uint8_t channel, int angle);
    // convert dh angle in radians to servo degree command
    int dhRadtoDeg(float dh_rad, int joint);
    // function to write all servos
    void writeServos();
    // get Dh transform
    void dhTransform(float theta, float a, float d, float alpha, BLA::Matrix<4,4>& H);
    // get current end effector position
    void forwardKinematics(float* q_in, float& x, float& y, float& z);
    // compute current jacobian 
    void computeJacobian(BLA::Matrix<3,3>& J);
    // compute DLS inverse
    Matrix<3,3> DLSinv(BLA::Matrix<3,3>& J);
    // carry out control step 
    void pointControlStep(float pd_x, float pd_y, float pd_z);
    // trajectory control step
    void trajectoryControlStep(float pd_x, float pd_y, float pd_z, float pddot_x, float pddot_y, float pddot_z);


  private:

    const float a[DOF] = {0.0f, L2, L3};
    const float d[DOF] = {L1, 0.0f, 0.0f};  // TRY changing 
    const float alpha[DOF] = {-M_PI/2, 0.0f, M_PI};

    const float Q_MIN[3] = { 0.0f,   -M_PI,   -M_PI/2}; // +- 90 for each
    const float Q_MAX[3] = { M_PI,    0.0f,    M_PI/2};

    // starting position
    const float start_x = START_X;
    const float start_y = START_Y ;
    const float start_z = START_Z;

    // max joint velocity
    const float qdot_max = QDOT_MAX;

};

// initialise servo driver
void RobotArm::init_servo_driver(){
  ServoDriver.begin();
  ServoDriver.setPWMFreq(50);
}

 // convert degrees to pulse count 
int RobotArm::angleToPulse(int angle){
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

    // set single servo angle 
void RobotArm::setServoAngle(uint8_t channel, int angle){
  ServoDriver.setPWM(channel, 0, angleToPulse(angle));
}

// convert dh angle in radians to servo degree command
int RobotArm::dhRadtoDeg(float dh_rad, int joint){

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
void RobotArm::writeServos() {

  for (int i = 0; i < 3; i++) {

    int deg = dhRadtoDeg(q[i], i);
    setServoAngle(i, deg);

  }
}


// function to compute single homogenous transform
void RobotArm::dhTransform(float theta, float a, float d, float alpha, Matrix<4,4>& H){
    H = {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
      sin(theta), cos(theta) * cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
      0, sin(alpha), cos(alpha), d,
      0, 0, 0, 1
  };
}


// function to get current end effector position
void RobotArm::forwardKinematics(float* q_in, float& x, float& y,float& z){

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

// function to compute current Jacobian
void RobotArm::computeJacobian(BLA::Matrix<3,3>& J){

  float x0, y0, z0;
  forwardKinematics(q, x0, y0, z0);  // current end effector pos.

  for (int i = 0; i<DOF; i ++){

    float q_pert[3] = {q[0], q[1], q[2]}; 
    q_pert[i] += DELTA;     // perturb each joint slightly

    float xp, yp, zp;
    forwardKinematics(q_pert, xp, yp, zp); // ee position after perturbation

    // jacobian rows
    J(0,i) = (xp - x0) / DELTA;  // dx/dqi
    J(1,i) = (yp - y0) / DELTA;  // dy/dqi
    J(2,i) = (zp - z0) / DELTA;  // dz/dqi

  }

}

// calculate DLS inverse
Matrix<3,3> RobotArm::DLSinv(Matrix<3,3>& J) {

  Matrix<3,3> Jt = ~J; //transpose
  Matrix<3,3> JJt   = J * Jt;

    // Add damping to diagonal
  JJt(0,0) += lambda * lambda;
  JJt(1,1) += lambda * lambda;
  JJt(2,2) += lambda * lambda;

  return Jt * Inverse(JJt);

}

// Carry out control step 
void RobotArm::pointControlStep(float pd_x, float pd_y, float pd_z){

  static int iter = 0;
  const int max_iter = MAX_ITER;

  // get current end effector position
  float px, py, pz;
  forwardKinematics(q, px, py, pz);

  //error
  float ex = pd_x - px;
  float ey = pd_y - py;
  float ez = pd_z - pz;
  float norm_error = sqrt(ex*ex + ey*ey + ez*ez);

  if(norm_error < pos_threshold) {

    if(!target_reached) {
      Serial.println("Target reached");
      target_reached = true;
    }

    return;

  }

  // give up if not converged after max iterations

  if(iter > max_iter){

    Serial.println("Failed to reach target - returning to start");
    target_reached = true; // stop trying 
    iter = 0;

    // go back to safe start pose
    q[0] = J0_START;
    q[1] = J1_START;
    q[2] = J2_START;
    writeServos();

    return;
  }

  iter++;

  // Jacobian
  Matrix<3,3> J;
  computeJacobian(J); //computeJacobian(q, J); if doesnt work need to rewrtie some stuff

  // dls inverse
  Matrix<3,3> Jinv = DLSinv(J);

  // control law
  Matrix<3,1> error_vector;
  error_vector(0) = ex;
  error_vector(1) = ey;
  error_vector(2) = ez;

  Matrix<3,1> qdot = Jinv *(Kp * error_vector);

  // integrate and clamp
  for(int i = 0; i < 3; i++) {
    q[i] = constrain(q[i] + qdot(i) * dt, Q_MIN[i], Q_MAX[i]);
  }


}

// Trajectory Control Step

void RobotArm::trajectoryControlStep(float pd_x, float pd_y, float pd_z, float pddot_x, float pddot_y, float pddot_z){

  // get current EE position
  float px, py, pz;
  forwardKinematics(q, px, py, pz);

  //error
  float ex = pd_x - px;
  float ey = pd_y - py;
  float ez = pd_z - pz;
  float norm_error = sqrt(ex*ex + ey*ey + ez*ez);

  Serial.print("pd: "); Serial.print(pd_x); Serial.print(", ");
                          Serial.print(pd_y); Serial.print(", ");
                          Serial.println(pd_z);
  Serial.print("EE: "); Serial.print(px);   Serial.print(", ");
                          Serial.print(py);   Serial.print(", ");
                          Serial.println(pz);
  Serial.print("err: "); Serial.println(norm_error);

  // jacobian and inverse
  Matrix<3,3> J;
  computeJacobian(J);
  Matrix<3,3> Jinv = DLSinv(J);

  // control law
  Matrix<3,1> pdot_cmd;
  pdot_cmd(0) = pddot_x + Kp * ex;
  pdot_cmd(1) = pddot_y + Kp * ey;
  pdot_cmd(2) = pddot_z + Kp * ez;

  Matrix<3,1> qdot_cmd;
  qdot_cmd = Jinv * pdot_cmd;

  // clamp joint velocities to prevent aggressive motion
  for (int i = 0; i < DOF; i++) {
      qdot_cmd(i) = constrain(qdot_cmd(i), -QDOT_MAX, QDOT_MAX);
  }

  // integrate
  for (int i = 0; i < DOF; i++) {
    q[i] = constrain(q[i] + qdot_cmd(i) * dt, Q_MIN[i], Q_MAX[i]);
  }

  // // check convergence only after trajectory is complete
  //   if (norm_error < pos_threshold) {
  //       if (!target_reached) {
  //           Serial.println("Trajectory complete - target reached");
  //           target_reached = true;
  //       }
  //   }

}



// ========== MISC FUNCTIONS =========================================

//function to print matrix (for debugging)
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


// immediate (basic) check if desired point is reachable
bool isReachable(float x, float y, float z) {
    
    // max reach = all links fully extended
    const float max_reach = L1 + L2;  // 29cm

    // distance from base origin to target in xy plane
    float xy_dist = sqrt(x*x + y*y);
    
    // distance from base origin to target in 3D
    // account for z offset from d1+d2 = 15.5cm base height
    float z_relative = z - 15.5f;
    float total_dist = sqrt(xy_dist*xy_dist + z_relative*z_relative);

    if (total_dist > max_reach) {
        return false;
    }

    // check z is above ground (arm cant go below base)
    if (z < 0.0f) {
        return false;
    }

    return true;
}

// ============================ OBJECTS ============================================

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40);
RobotArm arm(ServoDriver);

float vel[3];

// ============================= SETUP =============================================

void setup() {

  Serial.begin(115200);
  arm.init_servo_driver();

  // go to starting position
  arm.writeServos();
  delay(2000);

  // check end point reachable
  if(!isReachable(END_X, END_Y,END_Z)){
    Serial.println("End point not reachable ");
    while(true);
  }

  // compute constant velocity vector
  vel[0] = (END_X - START_X)/T_total;
  vel[1] = (END_Y - START_Y)/T_total;
  vel[2] = (END_Z - START_Z)/T_total;

  Serial.println("Starting trajectory");



}

void loop() {

    // run at 20ms intervals same as dt
  static unsigned long last_t = 0;
  static float t = 0.0f;

  unsigned long now = millis();

  if(now - last_t < (DT*1000)){  
    return;
  }  

  last_t = now;

  if(arm.target_reached){
    return;
  }

  // clamp time
  float t_clamped = min(t,T_total);
  
  // desired position at time t
  float pd[3] = {
    START_X + vel[0] * t_clamped,
    START_Y + vel[1] * t_clamped,
    START_Z + vel[2] * t_clamped
  };

  // desired velocity - constant then zero once reachedd
  float pddot[3] = {
        (t < T_total) ? vel[0] : 0.0f,
        (t < T_total) ? vel[1] : 0.0f,
        (t < T_total) ? vel[2] : 0.0f
    };

  arm.trajectoryControlStep(pd[0], pd[1], pd[2], pddot[0], pddot[1], pddot[2]);
  arm.writeServos();
  t += DT;

}