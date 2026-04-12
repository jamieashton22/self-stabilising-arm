#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
using namespace BLA;

// =============== MACROS and GLOBALS ===============================================================================================================================

#define SERVO_MIN 100
#define SERVO_MAX 550

// Starting position joint angles (joints 0-2 unchanged)
#define J0_START  M_PI/2
#define J1_START  -3*M_PI/4
#define J2_START  M_PI/3
// Joints 3 & 4 start at neutral (zero wrist deflection)
#define J3_START  0.0f
#define J4_START  0.0f

// Starting EE position
#define START_X  -3.0f
#define START_Y  -6.5f
#define START_Z   35.76f

// Link lengths (cm)
#define L1  12.5f
#define L2  14.5f
#define L3  14.5f
#define L4   1.0f   // wrist roll link length  -- measure and adjust
#define L5   7.0f   // wrist pitch link length -- measure and adjust

// Position controller
#define KP      50.0f
#define LAMBDA   0.01f
#define DT       0.02f

// Stabilisation controller
#define KP_STAB    5.0f   // proportional gain for wrist stabilisation
#define DT_STAB     0.02f  // stabilisation loop dt (matches main loop)

// Complementary filter coefficient (0=gyro only, 1=accel only; 0.98 typical)
#define ALPHA_CF    0.94f

// Joint 4/5 servo channels on the PCA9685
#define SERVO_CH_J3  3
#define SERVO_CH_J4  4

// Convergence
#define CONV_THRESHOLD  3.0f
#define ORI_THRESH  0.0872f //5 deg 

#define MAX_ITER        200

const float DELTA = 1e-4f;

// ============== IMU WRAPPER =============================================================================================================

class IMUHandler {

  public:
    Adafruit_MPU6050 mpu;

    float roll = 0.0f;  // radians, positive = right side down
    float pitch = 0.0f; // radians, positive = nose up

    bool begin(){
      if (!mpu.begin()) return false;
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      return true;
    }

    //complementary filter

    void update(float dt) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Accelerometer-derived angles (radians)
      float ax = a.acceleration.x;
      float ay = a.acceleration.y;
      float az = a.acceleration.z;

      float accel_roll  =  atan2f(ay, az);
      float accel_pitch =  atan2f(-ax, sqrtf(ay*ay + az*az));

      // Gyroscope rates (rad/s) — sensor axes may need sign flips depending on mounting
      float gx = g.gyro.x;   // roll rate
      float gy = g.gyro.y;   // pitch rate

      // Complementary filter
      roll  = ALPHA_CF * (roll  + gx * dt) + (1.0f - ALPHA_CF) * accel_roll;
      pitch = ALPHA_CF * (pitch + gy * dt) + (1.0f - ALPHA_CF) * accel_pitch;
    }
};

// ============== ROBOT ARM CLASS =============================================================================================================

class RobotArm {
  public:

    // --- DOF split ---
    static const int DOF_POS  = 3;   // position joints (0-2)
    static const int DOF_STAB = 2;   // stabilisation joints (3-4)
    static const int DOF      = DOF_POS + DOF_STAB;

    float q[DOF];  // all 5 joint angles

    float Kp      = KP;
    float lambda  = LAMBDA;
    float dt      = DT;
    float Kp_stab = KP_STAB;

    bool target_reached = false;
    const float pos_threshold = CONV_THRESHOLD;

    Adafruit_PWMServoDriver ServoDriver;

    float pd[3] = {START_X, START_Y, START_Z};

    // DH home angles (servo midpoint offsets) for all 5 joints
    //                  J0        J1        J2   J3    J4
    const float q_home_rad[5] = {M_PI/2, -M_PI/2, 0.0f, 0.0f, 0.0f};
    const float servo_dir[5]  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    RobotArm(Adafruit_PWMServoDriver _ServoDriver) {
      q[0] = J0_START;
      q[1] = J1_START;
      q[2] = J2_START;
      q[3] = J3_START;
      q[4] = J4_START;
      ServoDriver = _ServoDriver;
    }

    void init_servo_driver();
    int  angleToPulse(int angle);
    void setServoAngle(uint8_t channel, int angle);
    int  dhRadtoDeg(float dh_rad, int joint);
    void writeServos();

    void dhTransform(float theta, float a, float d, float alpha, BLA::Matrix<4,4>& H);

    // Forward kinematics using only the first 3 joints (position chain)
    void forwardKinematics(float* q_in, float& x, float& y, float& z);

    void computeJacobian(BLA::Matrix<3,3>& J);
    Matrix<3,3> DLSinv(BLA::Matrix<3,3>& J);

    // Position control step (joints 0-2)
    void controlStep();

    // Stabilisation step (joints 3-4) — driven by IMU
    // Target roll/pitch = 0 (keep EE level). Pass desired_roll/pitch if non-zero setpoint.
    void stabilisationStep(float measured_roll, float measured_pitch,
                           float desired_roll = 0.0f, float desired_pitch = 0.0f);

    void handleSerial();
    void printStatus();

  private:
    bool isReachable(float x, float y, float z);

    // DH parameters — position chain (joints 0-2)
    const float a_pos[DOF_POS]     = {0.0f,  L2,    L3};
    const float d_pos[DOF_POS]     = {L1,   3.0f,  0.0f};
    const float alpha_pos[DOF_POS] = {-M_PI/2, 0.0f, M_PI};

    // DH parameters — wrist chain (joints 3-4)
    // Joint 3: roll  — rotation about the outgoing Z axis of joint 2 frame
    //   a=L4, d=0, alpha=-pi/2  (redirects axis for pitch joint)
    // Joint 4: pitch — rotation about the new Y axis
    //   a=L5, d=0, alpha=0
    const float a_stab[DOF_STAB]     = {L4,       L5};
    const float d_stab[DOF_STAB]     = {0.0f,    0.0f};
    const float alpha_stab[DOF_STAB] = {-M_PI/2,  0.0f};

    // Joint limits
    const float Q_MIN[5] = { 0.0f,  -M_PI,   -M_PI/2,  -M_PI/2,  -M_PI/2};
    const float Q_MAX[5] = { M_PI,   0.0f,    M_PI/2,   M_PI/2,   M_PI/2};
};

// ============== ROBOT ARM FUNCTIONS =============================================================================================================

void RobotArm::init_servo_driver() {
  ServoDriver.begin();
  ServoDriver.setPWMFreq(50);
}

int RobotArm::angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void RobotArm::setServoAngle(uint8_t channel, int angle) {
  ServoDriver.setPWM(channel, 0, angleToPulse(angle));
}

int RobotArm::dhRadtoDeg(float dh_rad, int joint) {
  float delta_rad = dh_rad - q_home_rad[joint];
  float delta_deg = RAD_TO_DEG * delta_rad * servo_dir[joint];
  return constrain((int)(90.0f + delta_deg), 0, 180);
}

void RobotArm::writeServos() {
  // Joints 0-2: position servos on channels 0-2
  for (int i = 0; i < DOF_POS; i++) {
    setServoAngle(i, dhRadtoDeg(q[i], i));
  }
  // Joints 3-4: wrist servos on channels 3-4
  setServoAngle(SERVO_CH_J3, dhRadtoDeg(q[3], 3));
  setServoAngle(SERVO_CH_J4, dhRadtoDeg(q[4], 4));
}

void RobotArm::dhTransform(float theta, float a, float d, float alpha, Matrix<4,4>& H) {
  H = {cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
       sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
       0,           sin(alpha),             cos(alpha),            d,
       0,           0,                      0,                     1};
}

// Forward kinematics — position chain only (joints 0-2)
// This is used for IK. The wrist joints don't move the EE position significantly
// for a stabilisation wrist, so we decouple them.
void RobotArm::forwardKinematics(float* q_in, float& x, float& y, float& z) {
  Matrix<4,4> H = {1,0,0,0,
                   0,1,0,0,
                   0,0,1,0,
                   0,0,0,1};
  Matrix<4,4> Hi;
  for (int i = 0; i < DOF_POS; i++) {
    dhTransform(q_in[i], a_pos[i], d_pos[i], alpha_pos[i], Hi);
    H = H * Hi;
  }
  x = H(0,3);
  y = H(1,3);
  z = H(2,3);
}

void RobotArm::computeJacobian(BLA::Matrix<3,3>& J) {
  float x0, y0, z0;
  forwardKinematics(q, x0, y0, z0);

  for (int i = 0; i < DOF_POS; i++) {
    float q_pert[3] = {q[0], q[1], q[2]};
    q_pert[i] += DELTA;

    float xp, yp, zp;
    forwardKinematics(q_pert, xp, yp, zp);

    J(0,i) = (xp - x0) / DELTA;
    J(1,i) = (yp - y0) / DELTA;
    J(2,i) = (zp - z0) / DELTA;
  }
}

Matrix<3,3> RobotArm::DLSinv(Matrix<3,3>& J) {
  Matrix<3,3> Jt  = ~J;
  Matrix<3,3> JJt = J * Jt;
  JJt(0,0) += lambda * lambda;
  JJt(1,1) += lambda * lambda;
  JJt(2,2) += lambda * lambda;
  return Jt * Inverse(JJt);
}

bool RobotArm::isReachable(float x, float y, float z) {
  const float max_reach = L2 + L3;
  float z_relative = z - 15.5f;
  float total_dist  = sqrt(x*x + y*y + z_relative*z_relative);
  if (total_dist > max_reach) { Serial.println("REJECT: exceeds max reach"); return false; }
  if (z < 0.0f)               { Serial.println("REJECT: below base");        return false; }
  return true;
}

// Position control step — identical logic to original, only touches q[0..2]
void RobotArm::controlStep() {
  static int iter = 0;

  float px, py, pz;
  forwardKinematics(q, px, py, pz);

  float ex = pd[0] - px;
  float ey = pd[1] - py;
  float ez = pd[2] - pz;
  float norm_error = sqrt(ex*ex + ey*ey + ez*ez);

  if (norm_error < pos_threshold) {
    if (!target_reached) {
      Serial.println("Target reached");
      target_reached = true;
      iter = 0;
    }
    return;
  }

  if (iter > MAX_ITER) {
    Serial.println("Failed to reach target - returning to start");
    target_reached = true;
    iter = 0;
    q[0] = J0_START; q[1] = J1_START; q[2] = J2_START;
    writeServos();
    return;
  }

  iter++;

  Matrix<3,3> J;
  computeJacobian(J);
  Matrix<3,3> Jinv = DLSinv(J);

  Matrix<3,1> err;
  err(0) = ex; err(1) = ey; err(2) = ez;

  Matrix<3,1> qdot = Jinv * (Kp * err);

  for (int i = 0; i < DOF_POS; i++) {
    q[i] = constrain(q[i] + qdot(i) * dt, Q_MIN[i], Q_MAX[i]);
  }
}

// Stabilisation step — simple proportional control on joints 3 & 4.
// The wrist counteracts whatever roll/pitch the IMU reports so the
// end-effector platform stays at the desired orientation.
//
// Sign convention:
//   q[3] (roll joint)  — positive q[3] tilts EE to the right → negate roll error
//   q[4] (pitch joint) — positive q[4] tilts EE forward      → negate pitch error
//
// If your physical build inverts either axis, flip the corresponding sign below.
void RobotArm::stabilisationStep(float measured_roll,  float measured_pitch,
                                  float desired_roll,   float desired_pitch) {

  float roll_error  = desired_roll  - measured_roll;
  float pitch_error = desired_pitch - measured_pitch;

  if(roll_error < ORI_THRESH && pitch_error < ORI_THRESH){
    return;
  }

  // Proportional update — integrate with dt
  float dq3 =  Kp_stab * roll_error  * DT_STAB;
  float dq4 =  Kp_stab * pitch_error * DT_STAB;

  q[3] = constrain(q[3] + dq3, Q_MIN[3], Q_MAX[3]);
  q[4] = constrain(q[4] + dq4, Q_MIN[4], Q_MAX[4]);
}

void RobotArm::handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "H" || line == "h") {
    pd[0] = START_X; pd[1] = START_Y; pd[2] = START_Z;
    target_reached = false;
    Serial.println("\nReturning to start position");
    return;
  }

  if (line == "S" || line == "s") {
    q[0] = J0_START; q[1] = J1_START; q[2] = J2_START; q[3] = J3_START; q[4] = J4_START;
    target_reached = true;
    Serial.println("\nsafe");
    return;
  }


  if (line == "P" || line == "p") { printStatus(); return; }

  int first_space  = line.indexOf(' ');
  int second_space = line.indexOf(' ', first_space + 1);

  if (first_space == -1 || second_space == -1) { Serial.println("\nInvalid"); return; }

  float nx = line.substring(0, first_space).toFloat();
  float ny = line.substring(first_space + 1, second_space).toFloat();
  float nz = line.substring(second_space + 1).toFloat();

  Serial.print("\nTarget: ["); Serial.print(nx,2); Serial.print(", ");
  Serial.print(ny,2); Serial.print(", "); Serial.print(nz,2); Serial.println("]");

  if (isReachable(nx, ny, nz)) {
    pd[0] = nx; pd[1] = ny; pd[2] = nz;
    target_reached = false;
    Serial.println("Target accepted");
  } else {
    Serial.println("Target rejected");
  }
}

void RobotArm::printStatus() {
  float px, py, pz;
  forwardKinematics(q, px, py, pz);

  Serial.print("\nEE:     ["); Serial.print(px,2); Serial.print(", ");
  Serial.print(py,2); Serial.print(", "); Serial.print(pz,2); Serial.println("]");

  Serial.print("Target: ["); Serial.print(pd[0],2); Serial.print(", ");
  Serial.print(pd[1],2); Serial.print(", "); Serial.print(pd[2],2); Serial.println("]");

  Serial.print("Error:  ");
  Serial.println(sqrt(pow(pd[0]-px,2)+pow(pd[1]-py,2)+pow(pd[2]-pz,2)), 2);

  // Serial.print("q3 (roll):  "); Serial.print(RAD_TO_DEG * q[3], 1); Serial.println(" deg");
  // Serial.print("q4 (pitch): "); Serial.print(RAD_TO_DEG * q[4], 1); Serial.println(" deg");
}


// ========== MISC FUNCTIONS =========================================

template <int rows, int cols>
void printMatrix(BLA::Matrix<rows, cols> mat) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) { Serial.print(mat(i,j)); Serial.print("\t"); }
    Serial.println();
  }
  Serial.println();
}


// ============================ OBJECTS ============================================

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40);
RobotArm  arm(ServoDriver);
IMUHandler imu;


// ============================= SETUP =============================================

void setup() {
  Serial.begin(115200);
  arm.init_servo_driver();

  // Initialise IMU
  if (!imu.begin()) {
    Serial.println("ERROR: MPU6050 not found — check wiring (SDA/SCL, 3.3V, GND)");
    while (1) delay(10);  // halt — no point running without IMU
  }
  Serial.println("MPU6050 ready");

  Serial.println("\n=============== ROBOT ARM + STABILISED WRIST ======================");
  Serial.println("Going to start position...");
  delay(1000);
  arm.writeServos();

  // arm.setServoAngle(1,90);
}


// ============================= LOOP =============================================

void loop() {
  arm.handleSerial();

  static unsigned long last_t = 0;
  unsigned long now = millis();

  if (now - last_t < (unsigned long)(DT * 1000)) return;
  last_t = now;

  // 1. Update IMU (complementary filter uses same dt as control loop)
  imu.update(DT);

  // 2. Position control — joints 0-2
  if (!arm.target_reached) {
    arm.controlStep();
  }

  // 3. Stabilisation — joints 3-4 (always active once target is reached or not)
  //    Pass desired_roll / desired_pitch if you want a tilted setpoint.
  arm.stabilisationStep(imu.roll, imu.pitch);

  // 4. Write all 5 servos
  arm.writeServos();
}


// NOTES: Kp = 50
/* -3 6 35 works
home - (5,15,32) - (5 15 23) - (-5 15 23) - (-15 0 27)
-3 10 30 works, -3 10 30 -> 10 3 30
3 7 35 works -> 3 12 35 works
H -> -3, 15, 32 -> -3 15 25 -> 15 -3 25
H -> 15 -3 25 -> 15 -3 35 -> 15 -6 35 works
H -> -3 -15 25
H -> -3 6 32
H-? -6 3 32
*/