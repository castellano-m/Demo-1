/* Name: Madison Heeg and Andrew Rouze
 * Date: October 12, 2020
 *  
 * Title: Inner Loop Controller
 *  - Motor RW:   Channel A (yellow) = pin 2 (ISR)
 *                Channel B (white) = pin 12
 *                Vcc (blue) = 5V on Arduino
 *              
 *  - Motor LW:   Channel A (yellow) = pin 3(ISR) 
 *                Channel B (white) = pin 13
 *                Vcc (Blue) = pin 4 (set high)
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
#define B_LW            0           // channel B (white)
#define Vcc2            5            // secondary 5V supply for encoder

/* MOTOR SHIELD */
#define Enable          4            // HIGH - motor to receive current
#define STATUS          12           // Uknown pin on motor shield, DO NOT USE

/**************************************** GLOBAL CONSTANTS ***************************************/ 
#define batteryVoltage  8            // [V]      voltage available from battery
#define N               3200         // [counts] # of turns per one revolution, Pololu motor datasheet
#define micro           1000000      // [s]      conversion from micros to seconds  

/************************************* VARIABLE DECLARATIONS *************************************/ 
const double  fullRotation = 6.22;      // [rad]  approx angular position after full rotation

/* RIGHT MOTOR */
static int    R_AChannelNow;            // [1 or 0]   channel A of RW encoder 
static int    R_BChannelNow;            // [1 or 0]   channel B or RW encoder
static long          R_countNow = 0;           // [counts]   current encoder counts
static long          R_countPrev = 0; 
static double        R_angPosPrev = 0.0;       // [rad]      prev angular position
static double        R_angPosNow = 0.0;        // [rad]      current angular position
static double        R_angVelNow = 0.0;        // [rad/s]    angular velocity
static double        R_angVelPrev = 0.0;       // [rad/s]    angular velocity
static double        R_linVel = 0.0;           // [in/s]     linear velocity
static unsigned long R_timePrev = 0;           // [us]       prev time
static unsigned long R_timeNow = 0;            // [us]       current time
static unsigned long R_deltaT = 0;             // [us]       difference of current - prev time

/* LEFT MOTOR^ */
static int    L_AChannelNow;            // [1 or 0]   channel A of RW encoder 
static int    L_BChannelNow;            // [1 or 0]   channel B or RW encoder
static long          L_countNow = 0;           // [counts]   current encoder counts
static long          L_countPrev = 0;
static double        L_angPosPrev = 0.0;       // [rad]      prev angular position
static double        L_angPosNow = 0.0;        // [rad]      current angular position
static double        L_angVelNow = 0.0;        // [rad/s]    angular velocity
static double        L_angVelPrev = 0.0;       // [rad/s]    angular velocity
static double        L_linVel = 0.0;           // [in/s]      linear velocity
static unsigned long L_timePrev = 0;           // [us]       prev time
static unsigned long L_timeNow = 0;            // [us]       current time
static unsigned long L_deltaT = 0;             // [us]       difference of current - prev time


/* ROBOT */
/*  phi = angle w/ respect to x-axis */
    static double        x_prev = 0.0; static double y_prev = 0.0; static double phi_prev = 0.0;       // [in]   starting positions
    static double        x_now = 0.0; static double y_now = 0.0; static double phi_now = 0.0;          // [in]   current positions
static unsigned long timePrev = 0;                                                   // [s]    time exit main loop
static unsigned long timeNow = 0;                                                    // [s]    time enter main loop
static unsigned long deltaT = 0;                                                     // [s]    time diff b/t exit and enter

static double        scaleL_PWM = .94;                                                 

static double        J_linVel = 0.0;                                                 // [in/s]  linear velocity of robot as whole              
static double        J_rotVel = 0.0;                                                 // [rad/s] rotational velocity of robot as whole

//double x_posCalib = 4.16;                                                    // [in] calibration for front of camera to be 0 origin
//double y_posCalib = 4.16;                                                    // [in] calibration for front of camera to be 0 origin 

/*  R L 
 *  0 0 Forward   0 1 CCW   1 0 CW    1 1 Backwards
 *  CCW = negative rotational velocity
*/
bool          R_dir = 0;                // controls the direction of rotation
bool          L_dir = 0;                // controls the direction of rotation
double        R_motorVotlage = 0.0;     // [V]  control voltage from PID
double        L_motorVoltage = 0.0;     // [V]  control voltage from PID

/* CONTROLS */
/* put control, gain values, etc. here */
/*double forwardError = 0;
double fErrorInteg  = 0;
double fkProp       = 1;      //.044594;
double fkInteg      = .2;      //1.13395;
double fErrorRange  = 0.5;

double rotatError   = 0;
double rErrorInteg  = 0;
double rkProp       = 0.81361;
double rkInteg      = 7.94361;
double rErrorRange  = 0.2;*/

double rkProp       = 1;        // right motor proportional gain
double rkInteg      = 0.2;      // right motor integral gain
double rErrorRange  = PI/8;     // right motor error range

double lkProp       = 1;        // left motor proportional gain
double lkInteg      = 0.2;      // left motor integral gain
double lErrorRange  = PI/8;     // left motor error range

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const double  radius = 2.952;               // [in]  radius of wheels
const double  baseline = 10.827;            // [in]  width of robot

/************************************* DESIRED TASK VARIABLES ************************************/
double desiredXPos              = 60;  // inches
double loopTime                 = 0;
int    desiredCounts            = (desiredXPos/(2*PI*radius))*N;
double desireForwardAngPos      = (double)desiredCounts*2.0*PI/(double)N;
double angularPos_now           = 0;
double motorVoltage             = 0;
double dutyCycle                = 0;

double phi_des                  = 0;         // radians
double desireRotatAngPos        = 2*phi_des;  // radians

/************************************** FUNCTION PROTOTYPES **************************************/ 
void leftPiController(double desiredAng, double actualAng);                     // control left motor voltage
void rightPiController(double desiredAng, double actualAng);                    // control right motor voltage
void rotPiController(double desiredPhi, double rightAngNow, double leftAngNow); // control robot rotational velocity
void calculatePositionandVel();                                                 // update x, y, phi of robot
void updateRightWheel();                                                        // update angPos, angVel, and linVel of right wheel
void updateLeftWheel();                                                         // update angPos, angVel, and linVel of left wheel
void updateWheels();                                                            // update angPos, angVel, and linVel of both wheels

/********************************************* SETUP *********************************************/
void setup() {
  
  pinMode(Enable, OUTPUT); digitalWrite(Enable, HIGH);            // ensure motors get current
  pinMode(Vcc2, OUTPUT); digitalWrite(Vcc2, HIGH);                // secondary Vcc = 5V 
  pinMode(R_PWM, OUTPUT); pinMode(L_PWM, OUTPUT);                 // setting PWM as output
  pinMode(R_SIGN, OUTPUT); pinMode(L_SIGN, OUTPUT);               // setting signs as output
  
  pinMode(A_RW, INPUT_PULLUP); pinMode(B_RW, INPUT_PULLUP);       // setting channel A and B of right wheel to be pullup resistors
  pinMode(A_LW, INPUT_PULLUP); pinMode(B_LW, INPUT_PULLUP);       // setting channel A and B of left wheel to be pull up resistors

  attachInterrupt(digitalPinToInterrupt(A_RW), updateRW_countsISR, CHANGE);   // ISR monitor changes to right wheel
  attachInterrupt(digitalPinToInterrupt(A_LW), updateLW_countsISR, CHANGE);   // ISR monitor changes to left wheel
  
  Serial.begin(9600);                                                       // initialize serial monitor

  digitalWrite(R_SIGN, R_dir);  digitalWrite(L_SIGN, L_dir);                 // assign direction to motors
  analogWrite(R_PWM, 127);      analogWrite(L_PWM, scaleL_PWM*127);          // write 50% duty cycle to each motor

  Serial.print("START"); 
  
}

/********************************************* LOOP **********************************************/
void loop() {
  L_angVelNow = 0; 
  R_angVelNow = 0;     
  calculatePositionandVel();        /* update x, y, phi of robot */
  
  //Serial.print("D_Counts: "); Serial.print(desiredCounts); Serial.print("\t"); Serial.print("A_counts "); Serial.print(R_countNow); Serial.print("\t"); Serial.print("D_Xpos: "); Serial.print(desiredXPos); Serial.print("\t"); Serial.print("A_Phi "); Serial.print(phi_now);
  //Serial.print("\t"); Serial.print("R_angPrev: "); Serial.print(R_angPosPrev); Serial.print("\t"); Serial.print("\t"); Serial.print("R_angPos: "); Serial.print(R_angPosNow); Serial.print("\t"); Serial.print("R_angVel: "); Serial.print(R_angVelNow); Serial.print("\n");
  Serial.print(desireForwardAngPos); Serial.print("\t"); Serial.println(R_angPosNow); 
  
  /* implement PI controller */
  //angularPos_now = R_angPosNow;                             // PI controller uses right wheel's angular position for reference
  //if(phi_now == phi_des){                                   // if robot has been rotated to correct angle
  leftPiController(desireForwardAngPos, L_angPosNow); // drive to correct forward position
  rightPiController(desireForwardAngPos, R_angPosNow);
  /*} else {                                                  // else
    rotPiController(phi_des, L_angPosNow, R_angPosNow);   // rotate robot to correct angle      
  }*/

  /*forwardError = (double)desireAngPos - (double)angularPos_now;

  fErrorInteg += forwardError*(deltaT/(double)micro);
  motorVoltage = (forwardError*fkProp) + (fkInteg*fErrorInteg);

  if(abs(motorVoltage) > (double)batteryVoltage)  motorVoltage = (double)batteryVoltage;
  dutyCycle = ((abs(motorVoltage)/batteryVoltage)) * (double)255;

  digitalWrite(R_SIGN, R_dir);  digitalWrite(L_SIGN, L_dir);
  analogWrite(R_PWM, dutyCycle);  analogWrite(L_PWM, scaleL_PWM*dutyCycle); */
  

}

/********************************************* FUNCTIONS *****************************************/

/* control wheel velocity */
void leftPiController(double desiredAng, double actualAng){
  
  double leftError = (double)desiredAng - (double)actualAng;
  static double lErrorInteg = 0;

  lErrorInteg += (double)leftError*((double)deltaT/(double)micro);
  double leftMotorVoltage = ((double)leftError*(double)lkProp) + ((double)lkInteg*(double)lErrorInteg);

  if(abs(leftMotorVoltage) > (double)batteryVoltage)  leftMotorVoltage = (double)batteryVoltage;
  double leftDutyCycle = ((abs(leftMotorVoltage)/batteryVoltage)) * (double)255;

  
  if (motorVoltage < 0) {
    L_dir = 1; 
  } else {
    L_dir = 0;
  }
  
  digitalWrite(L_SIGN, L_dir);
  analogWrite(L_PWM, leftDutyCycle);
  
}

/* control right wheel velocity */
void rightPiController(double desiredAng, double actualAng){

  static double rErrorInteg = 0;
  double rightError = (double)desiredAng - (double)actualAng;

  rErrorInteg += (double)rightError*((double)deltaT/(double)micro);
  double rightMotorVoltage = ((double)rightError*(double)rkProp) + ((double)rkInteg*(double)rErrorInteg);

  if(abs(rightMotorVoltage) > (double)batteryVoltage)  rightMotorVoltage = (double)batteryVoltage;
  double rightDutyCycle = ((abs(rightMotorVoltage)/batteryVoltage)) * (double)255;

  
  if (motorVoltage < 0) {
    R_dir = 1; 
  } else {
    R_dir = 0;
  }
  
  digitalWrite(R_SIGN, R_dir);
  analogWrite(R_PWM, rightDutyCycle);
  
}

/* control robot rotation velocity */
void rotPiController(double desiredPhi, double rightAngNow, double leftAngNow){
  
  double rotationalError = (double)desiredPhi - (double)phi_now;
  double leftDesAng = 2*desiredPhi;     // convert robot radians to left wheel radians
  double rightDesAng = 2*desiredPhi;    // convert robot radians to right wheel radians

  if (desiredPhi >= PI){
    leftPiController(-leftDesAng, leftAngNow);
    rightPiController(rightDesAng, rightAngNow);
  } else {
    leftPiController(leftDesAng, leftAngNow);
    rightPiController(-rightDesAng, rightAngNow);    
  }
  
}

/* update right encoder count values */
void updateRW_countsISR() {
    R_timePrev = R_timeNow; 
    R_timeNow = micros();                 // capture the time enter ISR
    R_deltaT = R_timeNow - R_timePrev;    // calculate time since last ISR
    
    R_countPrev = R_countNow;             // set previous count value to  the current count value that has not been updated 

    R_AChannelNow = digitalRead(A_RW);    // read in Channel A 
    R_BChannelNow = digitalRead(B_RW);    // read in Channel B

    if (R_AChannelNow == R_BChannelNow) R_countNow +=2;
    else R_countNow -=2;

    updateRightWheel();                   // update right wheel velocities and positions
}

/* update left encoder count values */
void updateLW_countsISR() {
    L_timePrev = L_timeNow; 
    L_timeNow = micros();                           // capture the time enter ISR
    L_deltaT = L_timeNow - L_timePrev;              // calculate time since last ISR
  
    L_countPrev = L_countNow;             // set previous count value to  the current count value that has not been updated 

    L_AChannelNow = digitalRead(A_LW);    // read in Channel A 
    L_BChannelNow = digitalRead(B_LW);    // read in Channel B

    if (L_AChannelNow == L_BChannelNow) L_countNow -=2;
    else L_countNow +=2; 

    updateLeftWheel();                    // update right wheel velocities and positions
}

/* update right wheel velocities and positions */
void updateRightWheel() {
   
    if(R_countNow != R_countPrev) {
          R_angPosNow = (2.0*PI*(double)R_countNow)/(double)N;
          
          R_angVelNow = ((double)R_angPosNow - (double)R_angPosPrev)/((double)R_deltaT/(double)micro);
          R_linVel = (double)radius*(double)R_angVelNow;              // calculate linear velocity
          
          R_angPosPrev = R_angPosNow;             
          R_angVelPrev = R_angVelNow; 
    }
}


/* update left wheel velocities and positions */
void updateLeftWheel() {
    if(L_countNow != L_countPrev) {
          L_angPosNow = (2.0*PI*(double)L_countNow)/(double)N;
          
          L_angVelNow = ((double)L_angPosNow - (double)L_angPosPrev)/((double)L_deltaT/(double)micro);  // else, calculate angVel based on angPos
          L_linVel = (double)radius*(double)L_angVelNow;                                                // calculate linear velocity
 
          L_angPosPrev = L_angPosNow;             
          L_angVelPrev = L_angVelNow; 
    }
}

/* Calculate new position and angle of robot*/
void calculatePositionandVel() {
    
    timePrev = timeNow;
    timeNow = micros();                             // capture time enter loop
    deltaT = timeNow - timePrev;                    // calculate time since last enter

    //static double        x_prev = 0.0; static double y_prev = 0.0; static double phi_prev = 0.0;       // [in]   starting positions
    //static double        x_now = 0.0; static double y_now = 0.0; static double phi_now = 0.0;          // [in]   current positions
    
    /* calculate position and angle */
    x_now = x_prev + (((double)deltaT/(double)micro)*double(cos(phi_prev))*(R_linVel + L_linVel))/(2.0);
    y_now = y_prev + (((double)deltaT/(double)micro)*double(sin(phi_prev))*(R_linVel + L_linVel))/(2.0);
    phi_now = phi_prev + ((double)deltaT/(double)micro)*(radius/baseline)*(R_linVel-L_linVel);
  
    //Serial.print("R_linvel "); Serial.print(R_linVel); Serial.print("\t"); Serial.print("L_linvel "); Serial.print(L_linVel); Serial.print("\t"); Serial.print("x_pos: ");  Serial.println(x_now);
  
    /*robot linear and rotational velocities */
    J_linVel = ((double)R_linVel + (double)L_linVel)/(double)2.0;
    J_rotVel = ((double)L_linVel - (double)R_linVel)/(double)baseline; 
             
    /* set previous values to current values */
    x_prev = x_now;
    y_prev = y_now;
    phi_prev = phi_now; 
  
}