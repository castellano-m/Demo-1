/* Name: Madison Heeg and Andrew Rouze
   Date: October 30, 2020

   Title: Inner Loop Controller
    - Motor RW:   Channel A (yellow) = pin 2 (ISR)
                  Channel B (white) = pin 6
                  Vcc (blue) = 5V on Arduino

    - Motor LW:   Channel A (yellow) = pin 3(ISR)
                  Channel B (white) = pin 11
                  Vcc (Blue) = pin 4 (set high)
*/
/**************************************** PIN DEFINITIONS ****************************************/
/* RIGHT MOTOR - Motor B */
#define R_PWM           10           // PWM (black)
#define R_SIGN          8            // direction (red)
#define A_RW            2            // channel A, ISR (yellow)  
#define B_RW            6            // channel B (white)

/* LEFT MOTOR - Motor A */
#define L_PWM           9           // PWM (black)
#define L_SIGN          7            // direction (red)
#define A_LW            3            // channel A, ISR (orange)
#define B_LW            11           // channel B (white)
#define Vcc2            5            // secondary 5V supply for encoder

/* MOTOR SHIELD */
#define Enable          4            // HIGH - motor to receive current
#define STATUS          12           // Uknown pin on motor shield, DO NOT USE

/**************************************** GLOBAL CONSTANTS ***************************************/
#define batteryVoltage  8            // [V]      voltage available from battery
#define N               3200         // [counts] # of turns per one revolution, Pololu motor datasheet
#define micro           1000000      // [s]      conversion from micros to seconds 
#define milli           1000         // [s]      conversion factor from millis to seconds 

/************************************* VARIABLE DECLARATIONS *************************************/
const double  fullRotation = 6.22;      // [rad]  approx angular position after full rotation

/* RIGHT MOTOR */
static int           R_AChannelNow;            // [1 or 0]   channel A of RW encoder
static int           R_BChannelNow;            // [1 or 0]   channel B or RW encoder
static long          R_countNow = 0;           // [counts]   current encoder counts
static long          R_countPrev = 0;
static double        R_angPosPrev = 0.0;       // [rad]      prev angular position
static double        R_angPosNow = 0.0;        // [rad]      current angular position
static double        R_angVel = 0.0;        // [rad/s]    angular velocity
static double        R_linVel = 0.0;           // [in/s]     linear velocity

/* LEFT MOTOR^ */
static int           L_AChannelNow;            // [1 or 0]   channel A of RW encoder
static int           L_BChannelNow;            // [1 or 0]   channel B or RW encoder
static long          L_countNow = 0;           // [counts]   current encoder counts
static long          L_countPrev = 0;
static double        L_angPosPrev = 0.0;       // [rad]      prev angular position
static double        L_angPosNow = 0.0;        // [rad]      current angular position
static double        L_angVel = 0.0;            // [rad/s]    angular velocity
static double        L_linVel = 0.0;           // [in/s]      linear velocity
bool                 leftFix = true;


/* ROBOT */
/*  phi = angle w/ respect to x-axis */
static double        x_prev = 0.0; static double y_prev = 0.0; static double phi_prev = 0.0;       // [in]   starting positions
static double        x_now = 0.0; static double y_now = 0.0; static double phi_now = 0.0;          // [in]   current positions
static unsigned long timePrev = 0;                // [us]    time exit main loop
static unsigned long timeNow = 0;                 // [us]    time enter main loop
static unsigned long samplingTime = 30;           // [ms]   time diff b/t exit and enter

static double        J_linVel = 0.0;              // [in/s]  linear velocity of robot as whole
static double        J_rotVel = 0.0;              // [rad/s] rotational velocity of robot as whole

/*  R L
    0 0 Forward   0 1 CCW   1 0 CW    1 1 Backwards
    CCW = positive rotational velocity
*/
bool          R_dir = 0;                // controls the direction of rotation
bool          L_dir = 0;                // controls the direction of rotation
double        R_motorVotlage = 0.0;     // [V]  control voltage from PID
double        L_motorVoltage = 0.0;     // [V]  control voltage from PID

/* CONTROLS */
double rkProp       = 0.9335;    // right motor proportional gain
double rkInteg      = 0.0;      // right motor integral gain
double rErrorRange  = PI / 8;   // right motor error range

double lkProp       = 1.1;     // left motor proportional gain
double lkInteg      = 0.0;     // left motor integral gain
double lErrorRange  = PI / 8;  // left motor error range

double rkPropRot     = .5; double rkIntegRot    = 0.3;            // right motor gain values for rotational movement
double lkPropRot     = .5; double lkIntegRot    = 0.3;            // left motor gain values for rotational movement
double rotK[4] = {lkPropRot, lkIntegRot, rkPropRot, rkIntegRot};  // array to hold the rotational PI control gain values


/***************************************** MEASUREMENTS *****************************************/
const double  radius    = 2.952;        // [in]  radius of wheels
const double  baseline  = 10.827;       // [in]  width of robot

/**************************************** DEMO VARIABLES ****************************************/
double desiredXPos      = 60;           // [in]
double phi_des          = 0;           // [rad]

/********************************* CALCULATIONS FROM DEMO VARIABLES *****************************/
int    desiredCounts            = (desiredXPos / (2 * PI*radius)) * N;            // encoder counts based off distance to move in inches
double desireForwardAngPos      = (double)desiredCounts * 2.0 * PI / (double)N;   // angular position of wheels based off distance to move in inches
double leftrotError             = 0;                                              // rotational error
double rightrotError            = 0;

double desireRotatAngPos        = 1.5 * phi_des;       // scaled desired rotational angular position, experimentally found scaling factor
double rotErrorRange            = PI / 32;             // acceptable error range for rotational movement of the robot
static double rErrorInteg       = 0;                   // aggregate error (desired - actual)  of the right wheel for controls
static double lErrorInteg       = 0;                   // aggregate error (desired - actual) of light wheel for controls

bool   rotFlag                  = true;                // flag to indicate when to switch from rotational to forward control
int    numRotFlag               = 0;                   // count of how many times the flag was set true
/************************************** FUNCTION PROTOTYPES **************************************/
void motorPiController(double leftDesiredAng, double rightDesiredAng, double leftAngNow, double rightAngNow, double gainArr[4]);    // main motor PI controller
void rotPiController(double desiredPhi, double rightAngNow, double leftAngNow, double rotK[4]); // control robot rotational velocity
void printData();

/********************************************* SETUP *********************************************/
void setup() {
  Serial.begin(115200);       /* initialize serial monitor */

  pinMode(Enable, OUTPUT); digitalWrite(Enable, HIGH);            // ensure motors get current
  pinMode(Vcc2, OUTPUT); digitalWrite(Vcc2, HIGH);                // secondary Vcc = 5V
  pinMode(R_PWM, OUTPUT); pinMode(L_PWM, OUTPUT);                 // setting PWM as output
  pinMode(R_SIGN, OUTPUT); pinMode(L_SIGN, OUTPUT);               // setting signs as output

  pinMode(A_RW, INPUT_PULLUP); pinMode(B_RW, INPUT_PULLUP);       // setting channel A and B of right wheel to be pullup resistors
  pinMode(A_LW, INPUT_PULLUP); pinMode(B_LW, INPUT_PULLUP);       // setting channel A and B of left wheel to be pull up resistors

  attachInterrupt(digitalPinToInterrupt(A_RW), updateRW_countsISR, CHANGE);   // ISR monitor changes to right wheel
  attachInterrupt(digitalPinToInterrupt(A_LW), updateLW_countsISR, CHANGE);   // ISR monitor changes to left wheel

  if (leftFix) {                      // fix for left encoder channels initializing to weird values when left encoder is plugged into motor shield
    leftFix = false;                  // when working with the TA, we narrowed it down the issue to be due to two channels in the motor shield and the software
    L_angVel = 0.0;
    L_linVel = 0.0;
  }

}

/********************************************* LOOP **********************************************/
void loop() {
  timePrev = micros();

  /* update right wheel velocities and positions */
  R_angPosNow = (2.0 * PI * (double)R_countNow) / (double)N;
  R_angVel = ((double)R_angPosNow - (double)R_angPosPrev) / ((double)samplingTime / (double)milli);
  R_linVel = (double)radius * (double)R_angVel;

  /* update right wheel velocities and positions */
  L_angPosNow = (2.0 * PI * (double)L_countNow) / (double)N;

  L_angVel = ((double)L_angPosNow - (double)L_angPosPrev) / ((double)samplingTime / (double)milli);
  L_linVel = (double)radius * (double)L_angVel;

  //printData();  <- used for troubleshooting

  /* Calculate new position and angle of robot*/

  /* calculate position and angle */
  x_now = x_prev + (((double)samplingTime / (double)milli) * double(cos(phi_prev)) * (R_linVel + L_linVel)) / (2.0);
  y_now = y_prev + (((double)samplingTime / (double)milli) * double(sin(phi_prev)) * (R_linVel + L_linVel)) / (2.0);
  phi_now = phi_prev + ((double)samplingTime / (double)milli) * (radius / baseline) * (R_linVel - L_linVel);

  /*robot linear and rotational velocities */
  J_linVel = ((double)R_linVel + (double)L_linVel) / (double)2.0;
  J_rotVel = ((double)R_linVel - (double)L_linVel) / (double)baseline;


  /* implement PI controller */
  if (rotFlag == false) {                                     // if robot has been rotated to correct angle
    double K[4] = {lkProp, lkInteg, rkProp, rkInteg};         // array of gain values for forward control
    motorPiController(desireForwardAngPos, desireForwardAngPos, L_angPosNow, R_angPosNow, K);
  } else {                                                  // else
    rotPiController(desireRotatAngPos, R_angPosNow, L_angPosNow, rotK);       // rotate robot to correct angle
  }

  /* set previous values to current values */
  x_prev = x_now; y_prev = y_now; phi_prev = phi_now;
  R_angPosPrev = R_angPosNow; L_angPosPrev = L_angPosNow;
  R_countPrev = R_countNow; L_countPrev = L_countNow;

  timeNow = micros();

  delay((samplingTime) - ((timeNow - timePrev) / (unsigned long)milli));


}

/********************************************* FUNCTIONS *****************************************/

void motorPiController(double leftDesiredAng, double rightDesiredAng, double leftActualAng, double rightActualAng, double gainArr[4]) {

  /* Individual error of each wheel */
  double leftError = (double)leftDesiredAng - (double)leftActualAng;          // calculate angular position error left wheel
  double rightError = (double)rightDesiredAng - (double)rightActualAng;       // calculate angular position error right wheel

  lErrorInteg += (double)leftError * ((double)samplingTime / (double)milli);                                              // add left error over time segment to error integral
  double leftMotorVoltage = ((double)leftError * (double)gainArr[0]) + ((double)gainArr[1] * (double)lErrorInteg);  // set left motor voltage using PI control
  rErrorInteg += (double)rightError * ((double)samplingTime / (double)milli);                                             // add right error over time segment to error integral
  double rightMotorVoltage = ((double)rightError * (double)gainArr[2]) + ((double)gainArr[3] * (double)rErrorInteg); // set right motor voltage using PI control

  /* check if left PI output is saturated and set to battery voltage if so */
  if (leftMotorVoltage > (double)batteryVoltage)
    leftMotorVoltage = (double)batteryVoltage;
  else if (leftMotorVoltage < -(double)batteryVoltage)
    leftMotorVoltage = -(double)batteryVoltage;

  double leftDutyCycle = ((abs(leftMotorVoltage) / batteryVoltage)) * (double)255;                          // convert left motor voltage to PWM input

  /* check if right PI output is saturated and set to battery voltage if so */
  if (rightMotorVoltage > (double)batteryVoltage)
    rightMotorVoltage = (double)batteryVoltage;
  else if (rightMotorVoltage < -(double)batteryVoltage)
    rightMotorVoltage = -(double)batteryVoltage;

  double rightDutyCycle = ((abs(rightMotorVoltage) / batteryVoltage)) * (double)255;                        // convert right motor voltage to PWM input

  if (leftMotorVoltage < 0) {   // check for left wheel spin direction
    L_dir = 1;
  } else {
    L_dir = 0;
  }

  if (rightMotorVoltage < 0) {  // check for right wheel spin direction
    R_dir = 1;
  } else {
    R_dir = 0;
  }

  digitalWrite(L_SIGN, L_dir);          // assign left motor direction
  analogWrite(L_PWM, leftDutyCycle);    // send left motor PWM signal
  digitalWrite(R_SIGN, R_dir);          // assign left motor direction
  analogWrite(R_PWM, rightDutyCycle);   // send left motor PWM signal
}


/* control robot rotation velocity */
void rotPiController(double desiredPhi, double rightAngNow, double leftAngNow, double rotK[4]) {

  double rotGain[4] = {lkPropRot, lkIntegRot, rkPropRot, rkIntegRot};     // set gain values for rotational movmeent

  leftrotError = (double)desiredPhi - (double)leftAngNow;                 // calculate robot angular error
  rightrotError = (double)desiredPhi - (double)rightAngNow;               // calculate robot angular error

  if (leftrotError < rotErrorRange) {                                     // Check if left wheel within acceptable error range
    numRotFlag += 1;                                                      // The team realizes that we will need to change this to a
    if (numRotFlag >= 2) {                                                // more robust controls for future demos / projects
      rotFlag = false;                                                    // set flag to false, since rotational control no longer needed
      R_countNow = 0; R_countPrev = 0;                                    // reset global variables to zero for forward motion
      L_countNow = 0; L_countPrev = 0;
      rErrorInteg = 0; lErrorInteg = 0;
      return;
    }
  }
  motorPiController(desiredPhi, -desiredPhi, leftAngNow, rightAngNow, rotGain);

}

/* update right encoder count values */
void updateRW_countsISR() {
  R_AChannelNow = digitalRead(A_RW);    // read in Channel A
  R_BChannelNow = digitalRead(B_RW);    // read in Channel B

  if (R_AChannelNow == R_BChannelNow) R_countNow += 2;
  else R_countNow -= 2;
}

/* update left encoder count values */
void updateLW_countsISR() {

  L_AChannelNow = digitalRead(A_LW);    // read in Channel A
  L_BChannelNow = digitalRead(B_LW);    // read in Channel B

  if (L_AChannelNow == L_BChannelNow) L_countNow -= 2;
  else L_countNow += 2;

}


/*Helper function to print variables onto serial monitor */
void printData() {
  //Serial.print(micros()/(double)1000000);
  //Serial.print("\t"); Serial.print(desireForwardAngPos);
  //Serial.print("\t"); Serial.print(x_now);
  Serial.print("\t"); Serial.print(leftrotError);
  Serial.print("\t"); Serial.print(rightrotError);
  Serial.print("\t"); Serial.print(rotFlag);
  //Serial.print("\t"); Serial.print(R_angVel);
  //Serial.print("\t"); Serial.print(L_angVel);
  Serial.print("\t"); Serial.print(desireRotatAngPos);
  //Serial.print("\t"); Serial.print(phi_now);
  //Serial.print("\t"); Serial.print(rightrotError);
  /*Serial.print("\t"); Serial.print("R_angVel "); Serial.print(R_angVel);
    Serial.print("\t"); Serial.print("R_angPosPrev "); Serial.print(R_angPosPrev);
    Serial.print("\t"); Serial.print("R_angPosNow "); Serial.print(R_angPosNow);*/
  Serial.print("\n");
}
