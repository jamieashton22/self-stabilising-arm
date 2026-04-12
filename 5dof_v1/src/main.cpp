
/* 5 DOF VERSION 1 - using GROUND TRUTH - JUST POSITION - 05/04/26

Note: 

good trajectory
with position convergence 10
0 10 44
0 20 44
0 30 44
0 30 34
0 30 24
30 0 

*/

#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
using namespace BLA;


// =============== MACROS and GLOBALS ===============================================================================================================================

#define SERVO_A_MIN 100 // pulse count for 0 deg
#define SERVO_A_MAX 550 // pulse count for 180 deg
#define SERVO_B_MIN 100 // ADD
#define SERVO_B_MAX 600 // ADD

// starting position joint angles
#define J0_START M_PI/2
#define J1_START -3*M_PI/4
#define J2_START M_PI/3
#define J3_START 0// ADD
#define J4_START 0// ADD

// starting ee position coords
#define START_X 0    // ADD
#define START_Y -4
#define START_Z 44

// link lengths
#define L1 12.5 //cm
#define L2 14.5
#define L3 14.5
#define L4 1.0 
#define L5 7.0

#define KP_POS 10.0f // position gain
#define KP_ORI 5.0f // orientation gain
#define LAMBDA 0.05 // for DLS pseudoinv
#define DT 0.02 // dt

const float DELTA = 1e-4f; // difference step for jacobian computation

#define CONV_THRESHOLD 10.0 // convergence threshold (cm)
#define ORI_THRESHOLD 0.05f // in rad
#define MAX_ITER 100


// ============== ROBOT ARM CLASS =============================================================================================================

class RobotArm {
  
  public:

    static const int DOF = 5;
    float q[DOF];

    float Kp_pos = KP_POS;
    float Kp_ori = KP_ORI;
    float lambda = LAMBDA;
    float dt = DT;

    //DH angle in rad that corresponds to servo midpoint -- basically offsets
    const float q_home_rad[DOF] = {PI/2.0, -PI/2.0, 0.0f, 0.0f, 0.0f}; // ADD  
    const float servo_dir[DOF] = {1.0f,1.0f,1.0f,1.0f,1.0f};

    bool target_reached = false;
    const float pos_threshold = CONV_THRESHOLD;
    const float ori_threshold = ORI_THRESHOLD;

    Adafruit_PWMServoDriver ServoDriver;

    // currrent target position
    float pd[3] = {START_X, START_Y, START_Z};

    // desired orientation as rotation matrix 
    float Rd[3][3] = {
        { 0, 1, 0 },
        { 0.259, 0, 0.966 },
        { 0.966, 0, -0.259}
    };

    RobotArm(Adafruit_PWMServoDriver _ServoDriver) {
        q[0] = J0_START;
        q[1] = J1_START;
        q[2] = J2_START;
        q[3] = J3_START;
        q[4] = J4_START;
        ServoDriver = _ServoDriver;
    }

    // member functions

    //start servo driver
    void init_servo_driver();
    // convert degrees to pulse count
    int angleToPulse(int angle, bool isB);
    // set single servo angle 
    void setServoAngle(uint8_t channel, int angle, bool isB = false);
    // convert dh angle in radians to servo degree command
    int dhRadtoDeg(float dh_rad, int joint);
    // function to write all servos
    void writeServos();
    // get Dh transform
    void dhTransform(float theta, float a, float d, float alpha, BLA::Matrix<4,4>& H);
    // get current end effector position
    void forwardKinematics(float* q_in, float& x, float& y, float& z, float R[3][3]);
    // compute current jacobian 
    void computeJacobian(BLA::Matrix<5,5>& J);
    // compute DLS inverse
    Matrix<5,5> DLSinv(BLA::Matrix<5,5>& J);
    // carry out control step 
    void controlStep();

    void debugOrientation();

    // for serial input
    void handleSerial();
    // print end effector position and target
    void printStatus();

  private:

    bool isReachable(float x, float y, float z);

      // DH parameters 
    const float a_dh[DOF]     = { 0.0f,  L2,          L3,         L4,         L5   };
    const float d_dh[DOF]     = { L1,    3.0f,         0.0f,       3.0f,       0.0f };
    const float alpha_dh[DOF] = { -M_PI/2.0f, 0.0f,   M_PI,   M_PI/2.0f,  0.0f };

      // joint limits
    const float Q_MIN[DOF] = {  0.0f,    -M_PI,    -M_PI/2.0f,  -3*M_PI/4.0f,  -3*M_PI/4.0f }; //ADD
    const float Q_MAX[DOF] = {  M_PI,     0.0f,     M_PI/2.0f,   3*M_PI/4.0f,   3*M_PI/4.0f }; //ADD

};

// ============== MEMBER FUNCTIONS ================================================================

void RobotArm::init_servo_driver() {
    ServoDriver.begin();
    ServoDriver.setPWMFreq(50);
}

void RobotArm::setServoAngle(uint8_t channel, int angle, bool is270) {
    ServoDriver.setPWM(channel, 0, angleToPulse(angle, is270));
}
// map pulse to angl e
int RobotArm::angleToPulse(int angle, bool isB) {

    if (isB) {
        return map(angle, 0, 270, SERVO_B_MIN, SERVO_B_MAX);
    }
    return map(angle, 0, 180, SERVO_A_MIN, SERVO_A_MAX);
}

// Convert DH angle in radians to servo joint command 
int RobotArm::dhRadtoDeg(float dh_rad, int joint) {
    float delta_rad = dh_rad - q_home_rad[joint];
    float delta_deg = RAD_TO_DEG * delta_rad * servo_dir[joint];

    if (joint >= 3) {
        // servo B
        int servo_deg = (int)(135.0f + delta_deg);
        return constrain(servo_deg, 0, 270);
    } else {
        // big servos
        int servo_deg = (int)(90.0f + delta_deg);
        return constrain(servo_deg, 0, 180);
    }
}

void RobotArm::writeServos() {
    for (int i = 0; i < DOF; i++) {
        int deg    = dhRadtoDeg(q[i], i);
        bool isB = (i >= 3);
        setServoAngle(i, deg, isB);
    }
}

void RobotArm::dhTransform(float theta, float a, float d, float alpha,
                            BLA::Matrix<4,4>& H) {
    float ct = cos(theta), st = sin(theta);
    float ca = cos(alpha), sa = sin(alpha);
    H = { ct, -st*ca,  st*sa,  a*ct,
          st,  ct*ca, -ct*sa,  a*st,
         0.0f,    sa,     ca,     d,
         0.0f,  0.0f,   0.0f,  1.0f };
}

void RobotArm::forwardKinematics(float* q_in,
                                  float& x, float& y, float& z,
                                  float R[3][3]) {
    Matrix<4,4> H = { 1,0,0,0,
                      0,1,0,0,
                      0,0,1,0,
                      0,0,0,1 };
    Matrix<4,4> Hi;

    for (int i = 0; i < DOF; i++) {
        dhTransform(q_in[i], a_dh[i], d_dh[i], alpha_dh[i], Hi);
        H = H * Hi;
    }

    x = H(0,3);
    y = H(1,3);
    z = H(2,3);

    // Extract rotation matrix
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R[r][c] = H(r,c);
}

void RobotArm::computeJacobian(BLA::Matrix<5,5>& J) {

    float x0, y0, z0;
    float R0[3][3];
    forwardKinematics(q, x0, y0, z0, R0);

    for (int i = 0; i < DOF; i++) {

        float q_pert[DOF];
        for (int k = 0; k < DOF; k++) q_pert[k] = q[k];
        q_pert[i] += DELTA;

        float xp, yp, zp;
        float Rp[3][3];
        forwardKinematics(q_pert, xp, yp, zp, Rp);

        // Linear velocity columns
        J(0,i) = (xp - x0) / DELTA;
        J(1,i) = (yp - y0) / DELTA;
        J(2,i) = (zp - z0) / DELTA;

        // Angular velocity from skew-symmetric part of ΔR·R0ᵀ
        // ω_skew = (ΔR·R0ᵀ - I) / DELTA  →  extract roll & pitch rates
        // dR/dqi ≈ (Rp - R0) / DELTA
        // ω = skew_sym(dR · R0ᵀ)  →  [ω_x, ω_y, ω_z]
        // roll  = ω_x (index 3), pitch = ω_y (index 4)
        //
        // skew(M) extracts: ω_x = M(2,1), ω_y = M(0,2), ω_z = M(1,0)
        // where M = dR·R0ᵀ

        // Compute M = dRdq · R0ᵀ  (dRdq = (Rp-R0)/DELTA)
        float M[3][3] = {};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 3; k++)
                    M[r][c] += ((Rp[r][k] - R0[r][k]) / DELTA) * R0[c][k]; // R0ᵀ: swap r,c

        // Extract angular velocity from skew-symmetric part: 0.5*(M - Mᵀ)
        // ω_x = 0.5*(M[2][1] - M[1][2])  → roll rate
        // ω_y = 0.5*(M[0][2] - M[2][0])  → pitch rate
        // ω_z = 0.5*(M[1][0] - M[0][1])  → yaw  rate (dropped)
        J(3,i) = 0.5f * (M[2][1] - M[1][2]);   // roll
        J(4,i) = 0.5f * (M[0][2] - M[2][0]);   // pitch
    }
}

// 5×5 DLS pseudo-inverse: Jᵀ (J Jᵀ + λ²I)⁻¹
BLA::Matrix<5,5> RobotArm::DLSinv(BLA::Matrix<5,5>& J) {
    Matrix<5,5> Jt  = ~J;
    Matrix<5,5> JJt = J * Jt;

    for (int i = 0; i < 5; i++)
        JJt(i,i) += lambda * lambda;

    return Jt * Inverse(JJt);
}

bool RobotArm::isReachable(float x, float y, float z) {
    const float max_reach = L2 + L3 + L4 + L5;  // 37cm
    float z_relative = z - L1;
    float dist = sqrt(x*x + y*y + z_relative*z_relative);
    if (dist > max_reach) {
        Serial.println("REJECT: exceeds max reach");
        return false;
    }
    if (z < 0.0f) {
        Serial.println("REJECT: below base");
        return false;
    }
    return true;
}

void RobotArm::controlStep() {

    static int iter = 0;

    float px, py, pz;
    float Re[3][3];
    forwardKinematics(q, px, py, pz, Re);

    // Position error
    float ex = pd[0] - px;
    float ey = pd[1] - py;
    float ez = pd[2] - pz;
    float pos_err = sqrt(ex*ex + ey*ey + ez*ez);

    // Orientation error (roll & pitch only) — same cross-product formula as MATLAB
    // eo = 0.5 * Σ (Re_col × Rd_col)  for cols 0 and 1
    // We use cols 0 & 1 only (sufficient to constrain roll & pitch; yaw is free)
    float eo_x = 0.0f, eo_y = 0.0f;
    for (int c = 0; c < 2; c++) {
        // cross(Re[:,c], Rd[:,c])
        float cx = Re[1][c]*Rd[2][c] - Re[2][c]*Rd[1][c];
        float cy = Re[2][c]*Rd[0][c] - Re[0][c]*Rd[2][c];
        eo_x += 0.5f * cx;
        eo_y += 0.5f * cy;
    }
    float ori_err = sqrt(eo_x*eo_x + eo_y*eo_y);

    if (pos_err < pos_threshold && ori_err < ori_threshold) {
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
        q[3] = J3_START; q[4] = J4_START;
        writeServos();
        return;
    }

    iter++;

    // 5×5 Jacobian and DLS inverse
    Matrix<5,5> J;
    computeJacobian(J);
    Matrix<5,5> Jinv = DLSinv(J);

    // 5×1 error vector — separate gains for position and orientation
    Matrix<5,1> error_vec;
    error_vec(0) = Kp_pos * ex;
    error_vec(1) = Kp_pos * ey;
    error_vec(2) = Kp_pos * ez;
    error_vec(3) = Kp_ori * eo_x;
    error_vec(4) = Kp_ori * eo_y;

    Matrix<5,1> qdot = Jinv * error_vec;

    for (int i = 0; i < DOF; i++) {
        q[i] = constrain(q[i] + qdot(i) * dt, Q_MIN[i], Q_MAX[i]);
    }
}

void RobotArm::debugOrientation() {
    float px, py, pz;
    float Re[3][3];
    forwardKinematics(q, px, py, pz, Re);

    Serial.println("--- Rotation Matrix Re ---");
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            Serial.print(Re[r][c], 3); Serial.print("\t");
        }
        Serial.println();
    }

    // Also print each column vector so we can see where the EE axes point
    Serial.println("--- EE X-axis (col 0) ---");
    Serial.print(Re[0][0],3); Serial.print(" ");
    Serial.print(Re[1][0],3); Serial.print(" ");
    Serial.println(Re[2][0],3);

    Serial.println("--- EE Y-axis (col 1) ---");
    Serial.print(Re[0][1],3); Serial.print(" ");
    Serial.print(Re[1][1],3); Serial.print(" ");
    Serial.println(Re[2][1],3);

    Serial.println("--- EE Z-axis (col 2) ---");
    Serial.print(Re[0][2],3); Serial.print(" ");
    Serial.print(Re[1][2],3); Serial.print(" ");
    Serial.println(Re[2][2],3);
}

void RobotArm::handleSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line == "H" || line == "h") {
        pd[0] = START_X; pd[1] = START_Y; pd[2] = START_Z;
        target_reached = false;
        Serial.println("Returning to start position");
        return;
    }

    if (line == "P" || line == "p") {
        printStatus();
        return;
    }

    if(line =="D" || line == "d"){
        debugOrientation();
        return;
    }

    int s1 = line.indexOf(' ');
    int s2 = line.indexOf(' ', s1 + 1);
    if (s1 == -1 || s2 == -1) { Serial.println("Invalid"); return; }

    float nx = line.substring(0, s1).toFloat();
    float ny = line.substring(s1 + 1, s2).toFloat();
    float nz = line.substring(s2 + 1).toFloat();

    Serial.print("Target: ["); Serial.print(nx,2); Serial.print(", ");
    Serial.print(ny,2); Serial.print(", "); Serial.print(nz,2); Serial.println("]");

    if (isReachable(nx, ny, nz)) {
        pd[0] = nx; pd[1] = ny; pd[2] = nz;
        target_reached = false;
        Serial.println("Target accepted");
    } else {
        pd[0] = nx; pd[1] = ny; pd[2] = nz;
        Serial.println("Target rejected");
        target_reached = false;
    }
}

void RobotArm::printStatus() {
    float px, py, pz;
    float Re[3][3];
    forwardKinematics(q, px, py, pz, Re);

    Serial.print("EE pos:  ["); Serial.print(px,2); Serial.print(", ");
    Serial.print(py,2); Serial.print(", "); Serial.print(pz,2); Serial.println("]");

    Serial.print("Target:  ["); Serial.print(pd[0],2); Serial.print(", ");
    Serial.print(pd[1],2); Serial.print(", "); Serial.print(pd[2],2); Serial.println("]");

    float pos_err = sqrt(pow(pd[0]-px,2)+pow(pd[1]-py,2)+pow(pd[2]-pz,2));
    Serial.print("Pos err: "); Serial.println(pos_err, 3);

    // Print Euler angles (roll, pitch) for monitoring
    // Simple ZYX extraction from Re
    float pitch = asin(-Re[2][0]);
    float roll  = atan2(Re[2][1], Re[2][2]);
    Serial.print("Roll:    "); Serial.print(RAD_TO_DEG * roll,  2); Serial.println(" deg");
    Serial.print("Pitch:   "); Serial.print(RAD_TO_DEG * pitch, 2); Serial.println(" deg");
}

// ============== MISC ====================================================================

template <int rows, int cols>
void printMatrix(BLA::Matrix<rows, cols> mat) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            Serial.print(mat(i,j)); Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}

// ============== OBJECTS ================================================================

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40);
RobotArm arm(ServoDriver);

// ============== SETUP ==================================================================

void setup() {
    Serial.begin(115200);
    arm.init_servo_driver();

    Serial.println("===== 5-DOF ROBOT ARM =====");
    Serial.println("Going to start position...");
    delay(1000);
    arm.writeServos();
    Serial.println("now in loop");

    // // ── ASSEMBLY MODE ──────────────────────────────────────────
    // // Joints 0-2: 180° servos, midpoint = 90°
    // arm.setServoAngle(0, 90, false);
    // arm.setServoAngle(1, 90, false);
    // arm.setServoAngle(2, 90, false);

    // // Joints 3-4: 270° servos, midpoint = 135°
    // arm.setServoAngle(3, 135, true);
    // arm.setServoAngle(4, 135, true);

    // while(true); // halt — comment out everything below during assembly
    // // ── END ASSEMBLY MODE ──────────────────────────────────────
}

// ============== LOOP ===================================================================

void loop() {

    arm.handleSerial();

    static unsigned long last_t = 0;
    unsigned long now = millis();
    if (now - last_t < (unsigned long)(DT * 1000)) return;
    last_t = now;

    if (!arm.target_reached) {
        arm.controlStep();
        arm.writeServos();
    }
}