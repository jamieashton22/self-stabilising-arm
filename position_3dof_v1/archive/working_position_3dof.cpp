
/*

POSITION ONLY FOR 3 DOF

jacobian and forward kinematics functions work correctly
DLS inverse function works correctly 


Home position: servo angles = 90, 90, 90 - vertically upright, DH angles = M_PI/2, -M_PI/2, 0.0f which is x = -3, y - 0, z = 41.5
Non singular start position
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
float Kp = 5.0f;

const float lambda = 0.1; // for DLS pseudoinverse //TWEAK
const float dt = 0.02f; //20 ms loop  //TWEAK

const float DELTA = 1e-4f; // difference step for jacobian computation


// JOINT VARIABLES
// initial joint variables - safe start
// float q[3] =  {M_PI/2, -M_PI/4, M_PI/4};
// trying different safe start
// float q[3] =  {M_PI/2, -3*M_PI/4, 2*M_PI/3}; // high determinant
float q[3] =  {M_PI/2, -3*M_PI/4, M_PI/3};  // decent determinant, lets try it


const float Q_MIN[3] = { 0.0f,   -M_PI,   -M_PI/2}; // +- 90 for each
const float Q_MAX[3] = { M_PI,    0.0f,    M_PI/2};

// starting position
const float home_x = -3.0f;
const float home_y = -6.75f;
const float home_z = 36.51f;


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

// DESIRED TARGET POINT =====================================================================================================================

// float pd[3] = {home_x, home_y + 5.0f, home_z-5.0f}; 
// float pd[3] = {10.0f,10.0f,10.0f};
// float pd[3] = {home_x + 10.0f,  home_y,        home_z+100.0f};        // +x 
// float pd[3] = {home_x - 5.0f,  home_y,        home_z};        // -x
// float pd[3] = {home_x,         home_y - 10.0f, home_z};        // +y
// float pd[3] = {home_x,         home_y,        home_z + 5.0f}; // +z
// float pd[3] = {home_x,         home_y,        home_z - 5.0f}; // -z WORKED
// float pd[3] = {home_x + 5.0f,  home_y + 5.0f, home_z - 5.0f}; // diagonal WORKED
float pd[3] = {6.0f, 0.0f, 20.0f};

// ============================================================================================================================================================

// convergence threshold (cm)
const float pos_threshold = 5.0f;
bool target_reached = false; 


// CONTROLLER

void ControlStep() {

  static int iter = 0;
  const int MAX_ITER = 100; 
  // current end effector position
  float px, py, pz;
  forward_kinematics(q, px, py, pz); // calculate current pos. from fk


  //error
  float ex = pd[0] - px;
  Serial.print("\n ex: \n");
  Serial.println(ex);
  float ey = pd[1] - py;
  Serial.print("\n ey: \n");
  Serial.println(ey);
  float ez = pd[2] - pz;
  Serial.print("\n ez: \n");
  Serial.println(ez);
  float norm_error = sqrt(ex*ex + ey*ey + ez*ez);
  Serial.print("\n norm error \n");
  Serial.println(norm_error);

  // check converged

  if(norm_error < pos_threshold) {

    if(!target_reached) {
      Serial.println("Target reached");
      target_reached = true;
    }

    return;

  }

     // Give up if not converged after MAX_ITER steps
    if (iter > MAX_ITER) {
        Serial.println("WARNING: target unreachable, returning to start");
        target_reached = true;  // stop trying
        iter = 0;
        // optionally command back to safe start pose
        q[0] = M_PI/2;
        q[1] = -3*M_PI/4;
        q[2] = M_PI/3;
        writeServos();
        return;
    }

    iter++;

  // Jacobian
  Matrix<3,3> J;
  computeJacobian(q, J);

  // // Debug - print J before inversion
  // Serial.println("Jacobian:");
  // printMatrix(J);

  // dls inverse
  Matrix<3,3> Jinv = DLSinv(J);

  // control law - point to point so no pd_dot
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

bool isReachable(float x, float y, float z) {
    
    // max reach = all links fully extended
    const float L1 = 14.5f;
    const float L2 = 14.5f;
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

// ================== SETUP =====================================

void setup(){

  pwm.begin();
  pwm.setPWMFreq(50); // servos run at 50Hz
  Serial.begin(115200);

  // // go to home position
  // writeServos();  //{PI/2.0f, -PI/2.0, 0.0f};
  // delay(2000);

  // // go to non singular starting position
  // q[0] = M_PI/2;
  // q[1] = -M_PI/4;  
  // q[2] = M_PI/4;   
  writeServos(); 
  delay(2000);


  // float px,py,pz;
  // forward_kinematics(q,px,py,pz);
  // Serial.println("Starting EE position:");
  // Serial.print("x: "); Serial.println(px);
  // Serial.print("y: "); Serial.println(py);
  // Serial.print("z: "); Serial.println(pz);
  // // Serial.println("Moving to target...");
  // // delay(2000);

  // Matrix<3,3> J;
  // computeJacobian(q, J);
  // Serial.println("Jacobian at start pose:");
  // printMatrix(J);
  // Serial.println("Det: ");
  // Serial.print(Determinant(J));
  Serial.println("Checking if target is reachable");
  if(isReachable(pd[0],pd[1],pd[2]) == true){
    Serial.println("target accepted");
    target_reached = false;
  }
  else{
    Serial.println("Target rejected");
    target_reached = true;
  }


  

};

// =================== LOOP =====================================

void loop(){

  // run at 20ms intervals same as dt
  static unsigned long last_t = 0;
  unsigned long now = millis();

  if(now - last_t < (dt*1000)){  
    return;
  }  

  last_t = now;


  if(!target_reached) {
    ControlStep();  // updates q
    writeServos();  // writes q
    // Print current EE position for monitoring
    float px, py, pz;
    forward_kinematics(q, px, py, pz);
    Serial.print("EE: ");
    Serial.print(px); Serial.print(", ");
    Serial.print(py); Serial.print(", ");
    Serial.print(pz); Serial.print("  |  error: ");
    Serial.println(sqrt(pow(pd[0]-px,2)+pow(pd[1]-py,2)+pow(pd[2]-pz,2)));

  }

}

