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
#define START_Z 36.76

// link lengths
#define L1 12.5 //cm
#define L2 14.5
#define L3 14.5

#define KP 10.0 // controller gain
#define LAMBDA 0.01 // for DLS pseudoinv
#define DT 0.02 // dt

const float DELTA = 1e-4f; // difference step for jacobian computation

#define CONV_THRESHOLD 3.0 // convergence threshold (cm)
#define MAX_ITER 200


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

    // currrent target position
    float pd[3] = {START_X, START_Y, START_Z};

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
    void controlStep();

    // for serial input
    void handleSerial();   
    // print end effector position and target              
    void printStatus();                  

  private:

    bool isReachable(float x, float y, float z);

    const float a[DOF] = {0.0f, L2, L3};
    const float d[DOF] = {L1, 3.0f, 0.0f};  // TRY changing 
    const float alpha[DOF] = {-M_PI/2, 0.0f, M_PI};

    const float Q_MIN[3] = { 0.0f,   -M_PI,   -M_PI/2}; // +- 90 for each
    const float Q_MAX[3] = { M_PI,    0.0f,    M_PI/2};

    // starting position
    const float start_x = START_X;
    const float start_y = START_Y ;
    const float start_z = START_Z;

};

// ============== ROBOT ARM FUNCTIONS =============================================================================================================


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

// is reachable
bool RobotArm::isReachable(float x, float y, float z) {
    const float max_reach = L2 + L3;  // 29cm
    float z_relative = z - 15.5f;
    float total_dist = sqrt(x*x + y*y + z_relative*z_relative);
    if (total_dist > max_reach) {
        Serial1.println("REJECT: exceeds max reach");
        return false;
    }
    if (z < 0.0f) {
        Serial1.println("REJECT: below base");
        return false;
    }
    return true;
}

// Carry out control step 
void RobotArm::controlStep(){

  static int iter = 0;
  const int max_iter = MAX_ITER;

  // get current end effector position
  float px, py, pz;
  forwardKinematics(q, px, py, pz);

  //error
  float ex = pd[0] - px;
  float ey = pd[1] - py;
  float ez = pd[2]- pz;
  float norm_error = sqrt(ex*ex + ey*ey + ez*ez);

  if(norm_error < pos_threshold) {

    if(!target_reached) {
      Serial1.println("Target reached");
      target_reached = true;
      iter = 0;
    }

    return;

  }

  // // give up if not converged after max iterations

  if(iter > max_iter){

    Serial1.println("Failed to reach target - returning to start");
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


void RobotArm::handleSerial() {
    static char buf[32];
    static int buf_idx = 0;

    // read character by character - never blocks
    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n' || c == '\r') {
            if (buf_idx == 0) return;   // empty line
            buf[buf_idx] = '\0';        // null terminate
            buf_idx = 0;                // reset for next message

            // trim leading spaces
            char* input = buf;
            while (*input == ' ') input++;

            Serial.print("Buffer received: [");
            Serial.print(input);
            Serial.println("]");

            // H to home
            if ((input[0] == 'H' || input[0] == 'h') && input[1] == '\0') {
                pd[0] = START_X;
                pd[1] = START_Y;
                pd[2] = START_Z;
                target_reached = false;
                Serial1.println("Returning to start");
                return;
            }

            // P to print status
            if ((input[0] == 'P' || input[0] == 'p') && input[1] == '\0') {
                printStatus();
                return;
            }

            // parse three floats
            // float nx, ny, nz;
            // with this:
            String s = String(input);
            s.trim();
            int first_space  = s.indexOf(' ');
            int second_space = s.indexOf(' ', first_space + 1);

            if (first_space == -1 || second_space == -1) {
                Serial1.println("Invalid - use: x y z");
                return;
            }

            float nx = s.substring(0, first_space).toFloat();
            float ny = s.substring(first_space + 1, second_space).toFloat();
            float nz = s.substring(second_space + 1).toFloat();

            Serial1.print("Target: [");
            Serial1.print(nx, 2); Serial1.print(", ");
            Serial1.print(ny, 2); Serial1.print(", ");
            Serial1.print(nz, 2); Serial1.println("]");

            if (isReachable(nx, ny, nz)) {
                pd[0] = nx;
                pd[1] = ny;
                pd[2] = nz;
                target_reached = false;
                Serial1.println("Target accepted");
            } else {
                Serial1.println("Target rejected");
            }

            return;
        }

        // accumulate into buffer
        if (buf_idx < 31) {
            buf[buf_idx++] = c;
        }
    }
}
void RobotArm::printStatus() {

    float px, py, pz;
    forwardKinematics(q, px, py, pz);

    Serial1.println("                   ");
    Serial1.print("EE:  [");
    Serial1.print(px,2); Serial1.print(", ");
    Serial1.print(py,2); Serial1.print(", ");
    Serial1.print(pz,2); Serial1.println("]");

    Serial1.println("                   ");
    Serial1.print("Target: [");
    Serial1.print(pd[0],2); Serial1.print(", ");
    Serial1.print(pd[1],2); Serial1.print(", ");
    Serial1.print(pd[2],2); Serial1.println("]");
    Serial1.println("                   ");
    Serial1.print("Error: ");
    Serial1.println(sqrt(pow(pd[0]-px,2)+pow(pd[1]-py,2)+pow(pd[2]-pz,2)), 2);
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



// ============================ OBJECTS ============================================

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40);
RobotArm arm(ServoDriver);

// ========================= TARGET =============================================
// float pd[3] = {START_X, START_Y + 5.0f, START_Z - 10.0f}; // -z WORKED
// float pd[3] = {START_X, START_Y, START_Z - 5000.0f}; // -z WORKED
// float pd[3] = {-3, -20, 30}; // WORKS
// float pd[3] = {-20, 0, 30}; // works
// float pd[3] = {20, 0, 30}; // works
// float pd[3] = {17, 6, 35}; // works
// float pd[3] = {17, -5, 30}; // works
// float pd[3] = {15, -1, 30}; // works
// float pd[3] = {0, 0, 40}; // works
// float pd[3] = {15, 0, 30}; // works
// float pd[3] = {18, -5, 10}; // doesnt work
// float pd[3] = {18, -5, 15}; // doesnt work


// ================= TARGETS LIST ==============================================
/* with d = 3, start: pi/2, -3pi/4, pi/3, conv = 3cm, kp = 6 lambda = 0.01

-3 -6.5 30 works
-3 -20 30 works
-20 0 30  works
-20 5 30 works
-20 10 30 doesnt
-20 5 25 works
20 5 25 doesnt work 
17 6 35 doesnt work 
17 -6 35 works 
-17 6 35 doesnt work 
-16 3 34 works 
17 -5 30 works 
15 -1 30 works
15 1 30 doesnt 
15 4 30 works
0 0 40 doesnt
0 3 40 works
0 -3 40 works
3 0 40 doesnt work
-3 0 40 works
3 3 40 works
-3 -3 40 works
3 -3 40
10 -5 40 works 
-10 -5 40 works 
-10 5 40 works 
-10 5 35 works 
-3 10 36 works
-3 15 36 works
5 15 32 works 
10 15 23 works
-15 0 27 works
0 3 40 works 

demonstration route
home - (5,15,32) - (5 15 23) - (-5 15 23) - (-15 0 27)



*/




// ============================= SETUP =============================================

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial1.setTimeout(0);  // disable any timeout on Serial1 reads - belt and braces
  arm.init_servo_driver();

  // FOR ASSEMBLY
  // arm.setServoAngle(0,90);
  // arm.setServoAngle(1,90);
  // arm.setServoAngle(2,90);

  // go to starting position
  Serial1.println("                   ");
  Serial1.println("=============== ROBOT ARM TESTING PROGRAM ======================");
  Serial1.println("                   ");
  Serial1.println("Beginning arm test: ");
  Serial1.println("                   ");
  delay(1000);
  Serial1.println("Going to start position");
  Serial1.println("                   ");
  arm.writeServos();




}

void loop() {

  // get keyboard input
  arm.handleSerial();

  // run at 20ms intervals same as dt
  static unsigned long last_t = 0;
  unsigned long now = millis();

  if(now - last_t < (DT*1000)){  
    return;
  }  

  last_t = now;


  if(!(arm.target_reached)){

    arm.controlStep(); // update q
    arm.writeServos(); // write q 

  }

}