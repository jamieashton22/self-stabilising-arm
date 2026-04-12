#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
using namespace BLA;


// =============== MACROS and GLOBALS ===============================================================

#define SERVO_MIN     100
#define SERVO_MAX     550

#define SERVO_MIN_270 100   // ← TUNE
#define SERVO_MAX_270 600   // ← TUNE

#define J0_START  ( M_PI / 2.0f)
#define J1_START  (-3.0f * M_PI / 4.0f)
#define J2_START  ( M_PI / 3.0f)
#define J3_START  ( 0.0f)
#define J4_START  ( 0.0f)

#define START_X  -3.0f
#define START_Y  -6.5f
#define START_Z  36.76f

#define L1  12.5f
#define L2  14.5f
#define L3  14.5f
#define L4   1.0f
#define L5   7.0f

#define KP_POS    6.0f
#define KP_ORI    4.0f
#define LAMBDA    0.05f
#define DT        0.02f

const float DELTA = 1e-4f;

#define CONV_THRESHOLD  3.0f
#define ORI_THRESHOLD   0.05f
#define MAX_ITER        200
#define QDOT_MAX        0.5f   // rad/step clamp ← TUNE

// ── CIRCLE TRAJECTORY PARAMS ──────────────────────────────────────────────
#define CIRCLE_CX     -4.0f    // centre x (cm)  ← TUNE
#define CIRCLE_CY     0.0f    // centre y (cm)  ← TUNE
#define CIRCLE_CZ    30.0f    // height   (cm)  ← TUNE
#define CIRCLE_R     10.0f    // radius   (cm)  ← TUNE
#define CIRCLE_OMEGA  0.1f    // rad/s — start slow ← TUNE


// ============== ROBOT ARM CLASS =================================================================

class RobotArm {

public:

    static const int DOF = 5;
    float q[DOF];

    float Kp_pos = KP_POS;
    float Kp_ori = KP_ORI;
    float lambda  = LAMBDA;
    float dt      = DT;

    const float q_home_rad[DOF] = { M_PI/2.0f, -M_PI/2.0f, 0.0f, 0.0f, 0.0f };
    const float servo_dir[DOF]  = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };  // ← TUNE

    bool target_reached = false;
    bool circle_pending = false;
    const float pos_threshold = CONV_THRESHOLD;
    const float ori_threshold = ORI_THRESHOLD;

    Adafruit_PWMServoDriver ServoDriver;

    // current target position
    float pd[3] = { START_X, START_Y, START_Z };

    // feedforward velocity (zero for point-to-point, nonzero during circle)
    float pd_dot[3] = { 0.0f, 0.0f, 0.0f };

    // desired rotation — set to Re at start pose (your physically level config)
    float Rd[3][3] = {
        {  0.0f,   1.0f,  0.0f },
        {  0.259f, 0.0f,  0.966f },
        {  0.966f, 0.0f, -0.259f }
    };

    // circle state
    bool  circle_active = false;
    float circle_t      = 0.0f;   // elapsed time (s)

    RobotArm(Adafruit_PWMServoDriver _ServoDriver) {
        q[0] = J0_START; q[1] = J1_START; q[2] = J2_START;
        q[3] = J3_START; q[4] = J4_START;
        ServoDriver = _ServoDriver;
    }

    void init_servo_driver();
    int  angleToPulse(int angle, bool is270);
    void setServoAngle(uint8_t channel, int angle, bool is270);
    int  dhRadtoDeg(float dh_rad, int joint);
    void writeServos();

    void dhTransform(float theta, float a, float d, float alpha, BLA::Matrix<4,4>& H);
    void forwardKinematics(float* q_in, float& x, float& y, float& z, float R[3][3]);

    void computeJacobian(BLA::Matrix<5,5>& J);
    BLA::Matrix<5,5> DLSinv(BLA::Matrix<5,5>& J);

    // update pd (and pd_dot) each tick if circle is active
    void updateCircleTarget();

    void controlStep();
    void handleSerial();
    void printStatus();
    void debugOrientation();

private:

    bool isReachable(float x, float y, float z);

    const float a_dh[DOF]     = { 0.0f, L2,         L3,    L4,        L5   };
    const float d_dh[DOF]     = { L1,   3.0f,        0.0f,  3.0f,      0.0f };
    const float alpha_dh[DOF] = { -M_PI/2.0f, 0.0f,  M_PI,  M_PI/2.0f, 0.0f };

    const float Q_MIN[DOF] = {  0.0f,  -M_PI,    -M_PI/2.0f,  -3*M_PI/4.0f,  -3*M_PI/4.0f };
    const float Q_MAX[DOF] = {  M_PI,   0.0f,     M_PI/2.0f,   3*M_PI/4.0f,   3*M_PI/4.0f };
};


// ============== SERVO FUNCTIONS =================================================================

void RobotArm::init_servo_driver() {
    ServoDriver.begin();
    ServoDriver.setPWMFreq(50);
}

int RobotArm::angleToPulse(int angle, bool is270) {
    if (is270) return map(angle, 0, 270, SERVO_MIN_270, SERVO_MAX_270);
    return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void RobotArm::setServoAngle(uint8_t channel, int angle, bool is270) {
    ServoDriver.setPWM(channel, 0, angleToPulse(angle, is270));
}

int RobotArm::dhRadtoDeg(float dh_rad, int joint) {
    float delta_deg = RAD_TO_DEG * (dh_rad - q_home_rad[joint]) * servo_dir[joint];
    if (joint >= 3) return constrain((int)(135.0f + delta_deg), 0, 270);
    return constrain((int)(90.0f + delta_deg), 0, 180);
}

void RobotArm::writeServos() {
    for (int i = 0; i < DOF; i++)
        setServoAngle(i, dhRadtoDeg(q[i], i), i >= 3);
}


// ============== KINEMATICS ======================================================================

void RobotArm::dhTransform(float theta, float a, float d, float alpha, BLA::Matrix<4,4>& H) {
    float ct = cos(theta), st = sin(theta);
    float ca = cos(alpha), sa = sin(alpha);
    H = { ct, -st*ca,  st*sa,  a*ct,
          st,  ct*ca, -ct*sa,  a*st,
         0.0f,    sa,     ca,     d,
         0.0f,  0.0f,   0.0f,  1.0f };
}

void RobotArm::forwardKinematics(float* q_in, float& x, float& y, float& z, float R[3][3]) {
    Matrix<4,4> H = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    Matrix<4,4> Hi;
    for (int i = 0; i < DOF; i++) {
        dhTransform(q_in[i], a_dh[i], d_dh[i], alpha_dh[i], Hi);
        H = H * Hi;
    }
    x = H(0,3); y = H(1,3); z = H(2,3);
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

        J(0,i) = (xp - x0) / DELTA;
        J(1,i) = (yp - y0) / DELTA;
        J(2,i) = (zp - z0) / DELTA;

        // Angular velocity from skew-symmetric part of dR/dqi · R0ᵀ
        float M[3][3] = {};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 3; k++)
                    M[r][c] += ((Rp[r][k] - R0[r][k]) / DELTA) * R0[c][k];

        J(3,i) = 0.5f * (M[2][1] - M[1][2]);   // roll
        J(4,i) = 0.5f * (M[0][2] - M[2][0]);   // pitch
    }
}

BLA::Matrix<5,5> RobotArm::DLSinv(BLA::Matrix<5,5>& J) {
    Matrix<5,5> Jt  = ~J;
    Matrix<5,5> JJt = J * Jt;

    // Variable damping — raise lambda if near singular
    float trace = 0.0f;
    for (int i = 0; i < 5; i++) trace += JJt(i,i);
    Serial.print("trace: ");
    Serial.println(trace, 4);
    float lam = (trace < 100.0f) ? lambda * 10.0f : lambda;  // ← TUNE threshold

    for (int i = 0; i < 5; i++) JJt(i,i) += lam * lam;
    return Jt * Inverse(JJt);
}


// ============== CIRCLE TRAJECTORY ===============================================================

void RobotArm::updateCircleTarget() {
    if (!circle_active) return;

    circle_t += dt;

    float angle = CIRCLE_OMEGA * circle_t;

    // update target position — same formula as MATLAB
    pd[0] = CIRCLE_CX + CIRCLE_R * cos(angle);
    pd[1] = CIRCLE_CY + CIRCLE_R * sin(angle);
    pd[2] = CIRCLE_CZ;

    // analytical feedforward velocity (derivative of pd)
    pd_dot[0] = -CIRCLE_R * CIRCLE_OMEGA * sin(angle);
    pd_dot[1] =  CIRCLE_R * CIRCLE_OMEGA * cos(angle);
    pd_dot[2] =  0.0f;
}


// ============== CONTROL =========================================================================

void RobotArm::controlStep() {

    static int iter = 0;

    float px, py, pz;
    float Re[3][3];
    forwardKinematics(q, px, py, pz, Re);

    // position error
    float ex = pd[0] - px;
    float ey = pd[1] - py;
    float ez = pd[2] - pz;
    float pos_err = sqrt(ex*ex + ey*ey + ez*ez);

    // orientation error — roll & pitch only
    float eo_x = 0.0f, eo_y = 0.0f;
    for (int c = 0; c < 2; c++) {
        eo_x += 0.5f * (Re[1][c]*Rd[2][c] - Re[2][c]*Rd[1][c]);
        eo_y += 0.5f * (Re[2][c]*Rd[0][c] - Re[0][c]*Rd[2][c]);
    }
    float ori_err = sqrt(eo_x*eo_x + eo_y*eo_y);

    // convergence check — only applies in point-to-point mode
    if (!circle_active) {
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
            q[0]=J0_START; q[1]=J1_START; q[2]=J2_START;
            q[3]=J3_START; q[4]=J4_START;
            writeServos();
            return;
        }

        iter++;
    }

    // Jacobian and DLS inverse
    Matrix<5,5> J;
    computeJacobian(J);
    Matrix<5,5> Jinv = DLSinv(J);

    // error vector — weighted by separate gains
    Matrix<5,1> error_vec;
    error_vec(0) = Kp_pos * ex;
    error_vec(1) = Kp_pos * ey;
    error_vec(2) = Kp_pos * ez;
    error_vec(3) = Kp_ori * eo_x;
    error_vec(4) = Kp_ori * eo_y;

    // feedforward vector — position rows only, orientation ff is zero
    Matrix<5,1> ff_vec;
    ff_vec(0) = pd_dot[0];
    ff_vec(1) = pd_dot[1];
    ff_vec(2) = pd_dot[2];
    ff_vec(3) = 0.0f;
    ff_vec(4) = 0.0f;

    Matrix<5,1> qdot = Jinv * (ff_vec + error_vec);

    // clamp qdot — prevents freakout near singularities
    float qdot_norm = 0.0f;
    for (int i = 0; i < DOF; i++) qdot_norm += qdot(i)*qdot(i);
    qdot_norm = sqrt(qdot_norm);
    Serial.print("qdot_norm: ");
    Serial.println(qdot_norm, 4);

    if (qdot_norm > 50.0f) {  // ← TUNE
        if (circle_active) {
            Serial.println("Singular pose on circle - stopping");
            circle_active = false;
            pd_dot[0] = pd_dot[1] = pd_dot[2] = 0.0f;
            // snap pd back to current position so arm holds still
            pd[0] = px; pd[1] = py; pd[2] = pz;
        } else {
            Serial.println("Singular pose - returning to start");
            target_reached = true;
            iter = 0;
            q[0]=J0_START; q[1]=J1_START; q[2]=J2_START;
            q[3]=J3_START; q[4]=J4_START;
            writeServos();
        }
        return;
    }

    for (int i = 0; i < DOF; i++) {
        float clamped = constrain(qdot(i), -QDOT_MAX, QDOT_MAX);
        q[i] = constrain(q[i] + clamped * dt, Q_MIN[i], Q_MAX[i]);
    }
}


// ============== SERIAL HANDLING =================================================================

void RobotArm::handleSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // H — return home
    if (line == "H" || line == "h") {
        circle_active = false;
        pd_dot[0] = pd_dot[1] = pd_dot[2] = 0.0f;
        pd[0] = START_X; pd[1] = START_Y; pd[2] = START_Z;
        target_reached = false;
        Serial.println("Returning to start position");
        return;
    }

    // P — print status
    if (line == "P" || line == "p") {
        printStatus();
        return;

    }

    // D — debug orientation
    if (line == "D" || line == "d") {
        debugOrientation();
        return;
    }

    // C — start circle
    if (line == "C" || line == "c") {
        // move to circle start point first, then begin
        pd[0] = CIRCLE_CX + CIRCLE_R;  // angle=0 start point
        pd[1] = CIRCLE_CY;
        pd[2] = CIRCLE_CZ;
        pd_dot[0] = pd_dot[1] = pd_dot[2] = 0.0f;
        target_reached = false;
        circle_pending = true;        // wait until arm reaches start point
        circle_t       = 0.0f;
        Serial.println("Moving to circle start point...");
        return;
    }

    // S — stop circle
    if (line == "S" || line == "s") {
        circle_active = false;
        pd_dot[0] = pd_dot[1] = pd_dot[2] = 0.0f;
        pd[0] = START_X; pd[1] = START_Y; pd[2] = START_Z;
        target_reached = false;
        Serial.println("Circle stopped - returning home");
        return;
    }

    // x y z — point-to-point target
    int s1 = line.indexOf(' ');
    int s2 = line.indexOf(' ', s1 + 1);
    if (s1 == -1 || s2 == -1) { Serial.println("Invalid"); return; }

    float nx = line.substring(0, s1).toFloat();
    float ny = line.substring(s1+1, s2).toFloat();
    float nz = line.substring(s2+1).toFloat();

    Serial.print("Target: ["); Serial.print(nx,2); Serial.print(", ");
    Serial.print(ny,2); Serial.print(", "); Serial.print(nz,2); Serial.println("]");

    if (isReachable(nx, ny, nz)) {
        circle_active = false;
        pd_dot[0] = pd_dot[1] = pd_dot[2] = 0.0f;
        pd[0] = nx; pd[1] = ny; pd[2] = nz;
        target_reached = false;
        Serial.println("Target accepted");
    } else {
        Serial.println("Target rejected");
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

    float pitch = asin(-Re[2][0]);
    float roll  = atan2(Re[2][1], Re[2][2]);
    Serial.print("Roll:    "); Serial.print(RAD_TO_DEG * roll,  2); Serial.println(" deg");
    Serial.print("Pitch:   "); Serial.print(RAD_TO_DEG * pitch, 2); Serial.println(" deg");
    Serial.print("Circle:  "); Serial.println(circle_active ? "ACTIVE" : "OFF");
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
    Serial.println("--- EE Z-axis (col 2) ---");
    Serial.print(Re[0][2],3); Serial.print(" ");
    Serial.print(Re[1][2],3); Serial.print(" ");
    Serial.println(Re[2][2],3);
}

bool RobotArm::isReachable(float x, float y, float z) {
    const float max_reach = L2 + L3 + L4 + L5;
    float z_relative = z - L1;
    float dist = sqrt(x*x + y*y + z_relative*z_relative);
    if (dist > max_reach) { Serial.println("REJECT: exceeds max reach"); return false; }
    if (z < 0.0f)          { Serial.println("REJECT: below base");        return false; }
    return true;
}


// ============== MISC ============================================================================

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


// ============== OBJECTS =========================================================================

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40);
RobotArm arm(ServoDriver);


// ============== SETUP ===========================================================================

void setup() {
    Serial.begin(115200);
    arm.init_servo_driver();
    Serial.println("===== 5-DOF ROBOT ARM =====");
    Serial.println("Commands: H=home  P=status  D=debug  C=circle  S=stop  x y z=target");
    delay(1000);
    arm.writeServos();
    Serial.println("Going to start position...");
}


// ============== LOOP ============================================================================

void loop() {
    arm.handleSerial();

    static unsigned long last_t = 0;
    unsigned long now = millis();
    if (now - last_t < (unsigned long)(DT * 1000)) return;
    last_t = now;

    // once arm reaches the circle entry point, begin circling
    static bool pending_circle = false;

    if (pending_circle && arm.target_reached) {
        arm.circle_active = true;
        arm.circle_t      = 0.0f;
        pending_circle    = false;
        Serial.println("Circle started");
    }

    // set pending_circle when C command received
    // we signal this via a public flag on the arm
    if (arm.circle_pending) {
        pending_circle       = true;
        arm.circle_pending   = false;
    }

    arm.updateCircleTarget();

    if (!arm.target_reached || arm.circle_active) {
        arm.controlStep();
        arm.writeServos();
    }
}